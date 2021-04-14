# UW AUTO RC CAR

## Accessing the images and yaml files

Go to https://drive.google.com/file/d/1kOiKhfo7qd6BElwIHmfHwmFHLIqhgl-Y/view?usp=sharing to access the image and yaml files (images under data, yaml files under info)

## Accessing the models
The most important model (steering angle) is at : https://drive.google.com/file/d/1cs8T3HUsizHTC0zm8dkhlvgvIIon2LhA/view?usp=sharing

The rest of the models (for the other yaml parameters) are at: https://drive.google.com/file/d/1MUv4Jcoq_jNuhamSf8jjFe_2ePnxwoCL/view?usp=sharing

# Some helpful notes

## Machine Learning Documentation

### Here are some websites and courses that I found useful:
The machine learning course by Stanford on Coursera: https://www.coursera.org/learn/machine-learning

Keras API docs: https://keras.io/api/

Tutorial of a CIFAR-10 CNN: https://machinelearningmastery.com/how-to-develop-a-cnn-from-scratch-for-cifar-10-photo-classification/ (other tutorials on this website are also useful)

How to install Anaconda: https://docs.anaconda.com/anaconda/install/windows/ 

How to install Tensorflow: How to Install TensorFlow on Windows - Shawn Hymel (for CPU)

https://shawnhymel.com/1961/how-to-install-tensorflow-with-gpu-support-on-windows/ (for GPU)

How to install Keras (make sure to have python and tensorflow first): How to Install Keras | Liquid Web

### Setting up the machine learning environment (windows):
1.	Install Anaconda
2.	Create an environment within Anaconda
3.	Install Tensorflow (make sure it is working by printing the version)
4.	Install Keras
5.	If you have a GPU, make sure it is set up with tensorflow correctly
6.	Then add a way to edit code externally (ex. Jupyter Notebooks)
7.	Start coding!

 

### Diagram of our Convolutional Neural Network
 	
Our convolutional neural network was based off the one above (https://arxiv.org/pdf/1604.07316.pdf). The input plane had 3 filters and the following Conv2D layers had 24, 36, 48 and 64 filters, respectively. The activation function that was used for all of the layers was the relu function. 

