.. dig-into-apollo documentation master file, created by
   sphinx-quickstart on Fri May 27 23:43:12 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Introduction
===========================================


Dig into Apollo is mainly to help learning Apollo autopilot system. We first introduce the functions of each module in detail, and then analyzed the code of each module. If you like autonomous driving and want to learn it, let's start this project and discuss anything you want to know!


.. note::
   - If "git clone" on github is too slow, pls try apollo mirror.
   - If you have any questions, please feel free to ask in git issue.
   - If you need to add new documents or give suggestions, welcome to participate in git discussion.


.. toctree::
   :caption: 快速开始:
   :numbered: 3
   :maxdepth: 4
   :hidden:

   what_is_apollo/readme
   how_to_build/readme
   docker/readme


.. toctree::
   :caption: Contents:
   :numbered: 3
   :maxdepth: 4
   :hidden:

   cyber/readme
   modules/index
   performance/readme
   simulation/readme


.. toctree::
   :caption: 参考
   :numbered: 3
   :maxdepth: 4
   :hidden:

   library/readme
   papers/readme
   questions/readme


Getting Started
===========================================
If you are like me before and don’t know how to start the Apollo project.Here are some suggestions.

- First understand the basic module functions. If you are not clear about the general function of the module, it's difficult for you to understand what's the code doing. Here is an beginner level tutorial.

- Then you need to understand the specific methods according to the module, which will be documented in this tutorial. We will analyze the code in depth next. You can learn step by step according to our tutorial.I know it will be a painful process, especially when you are first learning Apollo. If you persist in asking questions and studying, it will become easier in 1-2 months.

- Last but not least, the method in Apollo is almost perfect, but there will be some problems. Try to implement and improve it, find papers, try the latest methods, and hone your skills. I believe you will enjoy this process.


How to learn?
===========================================
:Basic learning:

Watching some introductory tutorials will help you understand the Apollo autopilot system more quickly. I highly recommend tutorial
Try to ask some questions, read some blogs, papers, or go to github to add some issues.
If you don’t have a self-driving car, you can try to deploy a simulation environment, I highly recommend lgsvl. Its community is very friendly!
If there are any problems with the simulation environment, such as map creation, or some other problems, welcome to participate in this project Flycars. We will help as much as possible!

:Code learning:

First of all, you must understand c++. If you are not very familiar with it, I recommend the book "c++ primer". This book is very good, but a bit thick. If you just want to start quickly, then try to find some simple tutorial, here I recommend teacher Hou Jie
After understanding C++, it is best to have some basic understanding of the modules, which will help you to read the code. This has been explained many times.
Use code reading tools to help you read the code. I highly recommend vscode. It supports both Windows and Linux, and has a wealth of plug-ins, which can help you track codes, search and find call relationships.
Of course, there are many professional knowledge and professional libraries. I can’t repeat the best tutorials one by one here, but I can try to recommend some. Hongyi Li's deep learning, even a math tutorial 3Blue1Brown's math
Do some experiments with the simulator. We say "Make your hands dirty". You can't just watch, you need to try to modify some configurations and see if it takes effect, if possible, you can also try to answer some questions.
Remember that autonomous driving is still far from mature. Read some papers, I read a lot of papers, it help me a lot.

.. tip::
   Hope you have fun!


Contributing
===========================================
This project welcomes contributions and suggestions. Please see our contribution guidelines.


References & resources
===========================================
.. apollo: https://github.com/ApolloAuto/apollo
.. awesome-self-driving-car: https://github.com/daohu527/awesome-self-driving-car


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
