# Dig into Apollo - Performance ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

## Table of Contents
- [线程调度](#schedule)
- [Cgroups](#cgroups)
- [CPU亲和性](#cpu)
- [中断绑定](#interrupt)
- [linux性能优化](#linux)
  - [Perf安装](#perf)
  - [火焰图](#flame_graph)
- [Reference](#reference)

我们主要从以下几个方面来优化我们的系统，使得系统更加稳定，节省资源，同时能够又能保证任务的实时性。以下几个技术都是目前Apollo Cyber中采用的技术。

<a name="schedule" />

## 线程调度
由于linux操作系统提供了控制线程的API接口，cyber通过系统提供的API对进程的优先级和使用的资源进行调节。首先cyber将要求比较高的进程设置为实时进程，linux操作系统中实时进程的优先级最高，实时进程可以抢占其它线程，好处是能够保证实时进程在一定的时间内返回结果，这在自动驾驶控制系统中非常关键，试想一下如果需要发送一条指令给汽车，系统没有在规定的时间内响应，或者响应有延迟，就有可能导致车祸。  
linux对**实时进程**的调度有2种方式：  
1. SCHED_FIFO - 先到的进程优先执行，后到的进程需要等之前的进程执行完成之后再开始执行。  
2. SCHED_RR - 基于时间片轮转，先到的进程执行完成之后放到队列尾部，在队列中循环执行。

> 基于FIFO方式的平均等待时间和进程的顺序有关系，如果先到的进程执行时间很长，那么后到的进程等待时间就会变长；如果先到进程的执行时间很短，那么后到进程的等待时间就会变短。当然基于时间片轮转的方式就没有这个缺点，但是先到进程的执行时间会变长，因为基于轮转的，需要循环队列执行，那么先到进程需要等待其它进程的执行。所以需要根据不同的场景来选择不同的调度策略。

<a name="cgroups" />

## Cgroups
cgroups，名称源自控制组群（control groups）的简写，是Linux内核的一个功能，用来限制、控制与分离一个进程组群的资源（如CPU、内存、磁盘输入输出等)。  
cgroups的一个设计目标是为不同的应用情况提供统一的接口，从控制单一进程（像nice）到操作系统层虚拟化（像OpenVZ，Linux-VServer，LXC）。cgroups提供：  
* 资源限制：组可以被设置不超过设定的内存限制；这也包括虚拟内存。
* 优先级：一些组可能会得到大量的CPU或磁盘IO吞吐量。
* 结算：用来衡量系统确实把多少资源用到适合的目的上。
* 控制：冻结组或检查点和重启动。

> 利用cgroups技术，我们可以设置这一组进程的优先级，并且根据重要程度和进程类型分配不同的资源。例如给重要的进程组分配更多的CPU和内存，限制其他进程组的CPU和内存防止其影响系统性能等。

<a name="cpu" />

## CPU亲和性
CPU亲和性又叫Processor affinity或CPU pinning。现在的CPU都是多核心的，比如Apollo推荐的计算单元就是4核8线程，多核心CPU的好处是可以同时执行多个任务。现在假设多核CPU有以下场景，一个核上的任务很多，而另外的核心都是空闲状态，那么就会出现一个核累死，而其他的核都在等待的状态，这时候操作系统就想到了一种技术来解决这个问题，即CPU的负载均衡，当一个CPU核心上的任务很多，而其他CPU是空闲状态的时候，操作系统会把这个核上的任务迁移到其他核心，这样多核CPU的利用率就上来了。  
对整个系统来说，CPU负载均衡是一个好技术，但是对单个线程来说，就不是那么好了。线程迁移会导致额外的开销，比如当前的CACHE需要重新刷新，而且把重要的任务绑定到单独的核心上，可以保证这个任务的高效执行而不被打断。  
linux操作系统中通过"sched_setaffinity" API来设置线程的CPU亲和性，通过"sched_getaffinity"来获取线程的CPU亲和性。
```
#define _GNU_SOURCE             /* See feature_test_macros(7) */
#include <sched.h>

int sched_setaffinity(pid_t pid, size_t cpusetsize,
                      cpu_set_t *mask);

int sched_getaffinity(pid_t pid, size_t cpusetsize,
                      cpu_set_t *mask);
```


<a name="interrupt" />

## 中断绑定
中断绑定又叫smp_affinity，通过"cat /proc/interrupts"可以列出系统中每个 I/O 设备中每个 CPU 的中断数，处理的中断数，中断类型，以及注册为接收中断的驱动程序列表。系统通过"smp_affinity"可以指定多核CPU是否会响应这个中断，这在频繁有中断的系统中相当有用，比如CAN总线会频繁通过中断来传递传感器消息，如果没有绑定中断，那么系统中每个核心都可能被打断，如果这个核心上有任务在运行，那么CPU就会打断当前任务的执行，而去处理中断程序，从而带来中断上下文切换开销。如果我们把中断绑定到一个单独的核心上，让这个CPU核心去处理中断，而其它CPU核心则不会被频繁打断。  
smp_affinity 的默认值为 f，即可为系统中任意 CPU 提供 IRQ。将这个值设定为 1，如下，即表示只有 CPU 0 可以提供这个中断：  
```
# echo 1 >/proc/irq/32/smp_affinity
# cat /proc/irq/32/smp_affinity
1
```



<a name="linux" />

## linux性能优化
linux操作系统的"perf"命令可以采样一段时间内的系统调用，保存成文件之后再结合火焰图，可以查看当前系统各个进程对cpu的使用情况，火焰图中的横轴代表了CPU占用时间的比例，宽度越宽，代表该进程越耗时。火焰图的横轴是当前进程的调用栈，可以逐级查看每个调用栈和具体的耗时。 

<a name="perf" />

#### perf安装
ubuntu下执行如下命令安装perf:  
```
apt-get install linux-tools-common linux-tools-generic linux-tools-`uname -r`
``` 

<a name="flame_graph" />

#### 火焰图
安装成功之后可以执行"perf"命令来采样系统进程调用:  
```
// 先找到需要统计的apollo进程
sudo ps -ef  | grep apollo

// 查看到进行号之后，用进程号替换下面的PID，进行采样，采样频率为99HZ，采样时间为120秒
sudo perf record -F 99 -p PID -g -- sleep 120

// 输出perf文件
sudo perf script > out.perf
```
上述步骤就完成了对apollo进程的采样，并且输出了采样文件，下面我们通过生成火焰图来分析进程的调用状况。火焰图采用开源工具"FlameGraph"，执行如下命令：  
```
// 下载FlameGraph项目
git clone --depth 1 https://github.com/brendangregg/FlameGraph.git

// 折叠调用栈
FlameGraph/stackcollapse-perf.pl out.perf > out.folded

// 生成火焰图
FlameGraph/flamegraph.pl out.folded > out.svg

```	

最后把生成的"out.svg"文件在浏览器中打开，就可以点击并且查看对应的调用时间和调用栈，来分析系统耗时。火焰图如下，源文件在github上下载:  
[![Gregg4](https://github.com/daohu527/Dig-into-Apollo/blob/master/performance/Gregg4.svg)](https://github.com/daohu527/Dig-into-Apollo/blob/master/performance/Gregg4.svg)  
> 图片引用自阮一峰《如何读懂火焰图？》



<a name="reference" />

## Reference
[cgroups](https://zh.wikipedia.org/wiki/Cgroups)  
[Processor affinity](https://en.wikipedia.org/wiki/Processor_affinity)  
[sched_setaffinity](https://linux.die.net/man/2/sched_setaffinity)  
[SMP IRQ affinity](https://www.kernel.org/doc/Documentation/IRQ-affinity.txt)  
[4.3. 中断和 IRQ 调节](https://access.redhat.com/documentation/zh-cn/red_hat_enterprise_linux/6/html/performance_tuning_guide/s-cpu-irq)  
[perf](http://www.brendangregg.com/perf.html)  
[使用Perf和火焰图分析CPU性能](http://senlinzhan.github.io/2018/03/18/perf/)    
[如何读懂火焰图？](http://www.ruanyifeng.com/blog/2017/09/flame-graph.html)   