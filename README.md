# UW AUTO RC CAR

### Folder Structure
Below is the folder structure needed in the root of the notebook. The names with extensions (.png, .yaml, .ipynb) are files while the names without extensions are folders. The indentation is representative of the tree structure, if the strucutre is changed, make sure to change the code folder structure for root:
```
root
    -> data
        -> allInputImages.png
    -> info
        -> allInputYamls.yaml
    -> models
        -> checkpoints
        -> final
    -> test
        -> allValidationImages.png
        -> allValidationYamls.yaml
    -> Model Training.ipynb
```
## Accessing the models
My outputted models can be found at : https://drive.google.com/drive/folders/1UzmW-JChktQwJZk2Tn8MUZN_fjMN7F6x?usp=sharing

# Some helpful notes

## Machine Learning Documentation

### Here are some websites and courses that I found useful:
The machine learning course by Stanford on Coursera: https://www.coursera.org/learn/machine-learning

Keras API docs: https://keras.io/api/

Tutorial of a CIFAR-10 CNN: https://machinelearningmastery.com/how-to-develop-a-cnn-from-scratch-for-cifar-10-photo-classification/ (other tutorials on this website are also useful)

How to install Anaconda: https://docs.anaconda.com/anaconda/install/windows/ 

How to install Tensorflow:
1. (CPU) - https://shawnhymel.com/2034/how-to-install-tensorflow-on-windows/
2. (GPU Windows) - https://shawnhymel.com/1961/how-to-install-tensorflow-with-gpu-support-on-windows/
3. (GPU Ubuntu) - https://towardsdatascience.com/installing-tensorflow-gpu-in-ubuntu-20-04-4ee3ca4cb75d

How to install Keras (make sure to have python and tensorflow first): https://www.liquidweb.com/kb/how-to-install-keras/

Car Deep Learning Resources (Including theory and code - **Very Useful**)
1. (Part 4) - https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96\
2. (**Part 5**) - https://towardsdatascience.com/deeppicar-part-5-lane-following-via-deep-learning-d93acdce6110

### Setting up the machine learning environment (Windows and Ubuntu):
1.	Install Anaconda
2.	Create a virtual environment within Anaconda
3.	Install Tensorflow (make sure it is working by printing the version)
4.	Install Keras
5.	If you have a GPU, make sure you have CUDA and cuDNN installed correctly by following the tutorial
6.	Then add a way to edit code externally (ex. Jupyter Notebooks)
7.	Start coding!

### Getting started with a free online machine learning environment (Google Colab):
1. Make sure you have a Google Account
2. Open google colab and connect to a host (CPU/GPU/TPU)
3. Import the necessary libraries (tensorflow/keras)
4. Start coding!

### Diagram of our Convolutional Neural Network
Our convolutional neural network was based off the one above (https://arxiv.org/pdf/1604.07316.pdf). The input plane had 3 filters and the following Conv2D layers had 24, 36, 48 and 64 filters, respectively. The activation function that was used for all of the layers was the relu function. 

