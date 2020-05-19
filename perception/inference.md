## inference推理
深度学习模型包括训练和推理2个过程，训练模型的过程是通过设计神经网络，通过数据训练出模型的参数。而推理则是部署深度学习的过程，实际上目前训练深度学习主要用的是python语言，而部署的时候大部分会采用c++，并且通过gpu进行加速，而inference模块则实现了上述功能。  

inference主要实现了tensorflow, caffe, paddlepaddle3种框架的实现，并且通过cuda进行计算加速。  

#### caffe


#### paddlepaddle


#### tensorrt


#### 目录介绍
```
├── BUILD
├── caffe // caffe框架
├── inference.cc // 定义了推理接口
├── inference_factory.cc // 推理工厂，用来创建推理器
├── inference_factory.h
├── inference_factory_test.cc
├── inference.h
├── inference_test.cc
├── inference_test_data  // 推理测试数据
├── layer.cc // 定义了layer接口
├── layer.h
├── layer_test.cc
├── operators // 算子？？？
├── paddlepaddle  // paddlepaddle框架
├── tensorrt // tensorrt框架
├── test
├── tools // yolo,lane等的例子
└── utils // cuda加速工具
```


#### CreateInferenceByName
CreateInferenceByName可以创建3种形式的推理器caffe, paddlepaddle, tensorrt。这里的tensorrt就是英伟达的加速库吗？也就是说如果是其它模型则采用tensorrt部署，caffe和paddlepaddle则采用这2种高级别的api部署？  
```c++
Inference *CreateInferenceByName(const std::string &name,
                                 const std::string &proto_file,
                                 const std::string &weight_file,
                                 const std::vector<std::string> &outputs,
                                 const std::vector<std::string> &inputs,
                                 const std::string &model_root) {
  if (name == "CaffeNet") {
    return new CaffeNet(proto_file, weight_file, outputs, inputs);
  } else if (name == "RTNet") {
    return new RTNet(proto_file, weight_file, outputs, inputs);
  } else if (name == "RTNetInt8") {
    return new RTNet(proto_file, weight_file, outputs, inputs, model_root);
  } else if (name == "PaddleNet") {
    return new PaddleNet(proto_file, weight_file, outputs, inputs);
  }
  return nullptr;
}
```

3种推理模型分别在caffe, paddlepaddle, tensorrt目录中，其中有用到cuda进行加速，其中".cu"是cuda对C++的扩展。  

