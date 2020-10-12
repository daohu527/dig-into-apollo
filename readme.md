# Dig into Apollo ![GitHub](https://img.shields.io/github/license/daohu527/Dig-into-Apollo.svg?style=popout)

Dig into Apollo is mainly to help learning [Apollo](https://github.com/ApolloAuto/apollo) autopilot system. We first introduce the functions of each module in detail, and then analyzed the code of each module. If you like autonomous driving and want to learn it, let's start this project and discuss anything you want to know!


## Friendly tips

If "git clone" on github is too slow, pls try apollo **[mirror](https://gitee.com/baidu/apolloauto)**. If you need more help and want to learning together, please add WeChat official accounts `fzyd006`.  


## Table of Contents

- [What's apollo](what_is_apollo)
- [How to build](how_to_build)
- [Code learning](code_learning)
    - [cyber](cyber)
    - [docker](docker)
    - [modules](modules)
        - [audio](modules/audio)
        - [bridge](modules/bridge)
        - [canbus](modules/canbus)
        - [drivers](modules/drivers)
        - [dreamview](modules/dreamview)   
        - [map](modules/map)
        - [localization](modules/localization)
        - [perception](modules/perception)
        - [prediction](modules/prediction)
        - [routing](modules/routing)
        - [planning](modules/planning)
        - [control](modules/control)
        - [transform](modules/transform)
        - [tools](modules/tools)
        - [v2x](modules/v2x)
- [performance](performance)
- [simulation](simulation)
- [library](library)
- [papers](papers)
- [questions](questions)


## Getting Started

If you are like me before and don’t know how to start the Apollo project. Here are some suggestions.  

1. First understand the basic module functions. If you are not clear about the general function of the module, it's difficult for you to understand what's the code doing. Here is an beginner level [tutorial](https://apollo.auto/devcenter/coursetable_cn.html?target=1)  
2. Then you need to understand the specific methods according to the module, which will be documented in this tutorial. We will analyze the code in depth next. You can learn step by step according to our tutorial.  
3. I know it will be a painful process, especially when you are first learning Apollo. If you persist in asking questions and studying, it will become easier in 1-2 months. 
4. Last but not least, the method in Apollo is almost perfect, but there will be some problems. Try to implement and improve it, find papers, try the latest methods, and hone your skills. I believe you will enjoy this process. 


## How to learn?

**Basic learning**  

1. Watching some introductory tutorials will help you understand the Apollo autopilot system more quickly. I highly recommend [tutorial](https://apollo.auto/devcenter/coursetable_cn.html?target=1)  
2. Try to ask some questions, read some blogs, papers, or go to github to add some [issues](https://github.com/ApolloAuto/apollo/issues).  
3. If you don’t have a self-driving car, you can try to deploy a simulation environment, I highly recommend [lgsvl](https://github.com/lgsvl/simulator). Its community is very friendly!  
4. If there are any problems with the simulation environment, such as map creation, or some other problems, welcome to participate in this project [Flycars](https://github.com/Flycars). We will help as much as possible!  

**Code learning**  

1. First of all, you must understand c++. If you are not very familiar with it, I recommend the book **"c++ primer"**. This book is very good, but a bit thick. If you just want to start quickly, then try to find some simple tutorial, here I recommend teacher [Hou Jie](https://search.bilibili.com/all?keyword=%E4%BE%AF%E6%8D%B7)  
2. After understanding C++, it is best to have some basic understanding of the modules, which will help you to read the code. This has been explained many times.  
3. Use code reading tools to help you read the code. I highly recommend [vscode](https://code.visualstudio.com/). It supports both Windows and Linux, and has a wealth of plug-ins, which can help you track codes, search and find call relationships.  
4. Of course, there are many professional knowledge and professional libraries. I can’t repeat the best tutorials one by one here, but I can try to recommend some. [Hongyi Li's deep learning](https://www.bilibili.com/video/BV1JE411g7XF?p=1), even a math tutorial [3Blue1Brown's math](https://space.bilibili.com/88461692/)  
5. Do some experiments with the simulator. We say "Make your hands dirty". You can't just watch, you need to try to modify some configurations and see if it takes effect, if possible, you can also try to answer some questions.  
6. Remember that autonomous driving is still far from mature. Read some papers, I read a lot of [papers](https://github.com/daohu527/awesome-self-driving-car#papers-blogs), it help me a lot.  

Hope you have fun!  

## Contributing
This project welcomes contributions and suggestions. Please see our contribution guidelines.  


## References & resources
- [apollo](https://github.com/ApolloAuto/apollo)  
- [awesome-self-driving-car](https://github.com/daohu527/awesome-self-driving-car)    
