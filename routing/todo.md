1. 为什么会出现找不到节点的情况？？？
节点没有连接，所以需要找到节点附近最近的路段。

2. Astar算法
http://www-cs-students.stanford.edu/~amitp/gameprog.html#Paths  

http://theory.stanford.edu/~amitp/GameProgramming/  

https://www.geeksforgeeks.org/a-search-algorithm/  

http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html  


3. "RoutingComponent"类继承至"cyber::Component"，并且申明为"public"方式，"cyber::Component"是一个模板类，它定义了"Initialize"和"Process"方法。  
```
template <typename M0>
class Component<M0, NullType, NullType, NullType> : public ComponentBase {
 public:
  Component() {}
  ~Component() override {}
  bool Initialize(const ComponentConfig& config) override;
  bool Process(const std::shared_ptr<M0>& msg);

 private:
  virtual bool Proc(const std::shared_ptr<M0>& msg) = 0;
};
```
// todo 模板方法中为虚函数，而继承类中为公有方法？为什么？
