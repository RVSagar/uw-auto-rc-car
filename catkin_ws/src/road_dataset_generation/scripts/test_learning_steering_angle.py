#!/usr/bin/env python

import os
import cv2
import numpy as np
import math
import random
import yaml
import uuid
import rospkg
import pickle as pk
import sklearn.model_selection

import matplotlib.pyplot as plt

import tensorflow as tf
from tensorflow.python.client import device_lib
import keras.models
from keras.models import Sequential
from keras.layers import Conv2D
from keras.layers import MaxPooling2D
from keras.layers import Dense
from keras.layers import Flatten
from keras.layers import LeakyReLU
from keras.layers import Dropout
from keras.optimizers import SGD, Adam
from keras.callbacks import EarlyStopping, ModelCheckpoint

print(device_lib.list_local_devices())

PREPROCESS_VERSION = 5

OUT_HEIGHT=120
OUT_WIDTH=160

def arrange_data(dataset, name):
    print("Arranging: %s" % name)

    X = [dat[0] for dat in dataset]
    Y = [dat[1] for dat in dataset]
    D = [dat[2] for dat in dataset]

    X = np.array(X)
    X = X.reshape([-1,OUT_HEIGHT,OUT_WIDTH,1])

    Y = np.array(Y)
    Y = Y.reshape([-1, 1])
    print("  X: " + str(X.shape))
    print("  Y: " + str(Y.shape))
    return X, Y, D


def preprocess_data(X, D):
    grey = cv2.cvtColor(X, cv2.COLOR_BGR2GRAY)

    # HEIGHT = 480
    # WIDTH = 640
    small = cv2.resize(grey, (OUT_WIDTH, OUT_HEIGHT))
    #cv2.imshow('image',small)
    #cv2.waitKey(0)

    small = small/255.0 - 0.5
    

    # Y = np.array([[D['X0']],
    #               [D['YAW0']],
    #               [D['inv_rad']]])

    Y = np.array([
                    # [D['X0']],
                    # [D['YAW0']],
                    # [D['inv_rad']],
                    [D['steering_command']]
                  
                 ])
    
    return (small, Y, D)

def load_dataset(pkg_dir, settings, sub_dir):
    dataset_dir = pkg_dir + "/output/" + sub_dir + "/"
    data_dir = dataset_dir + "data/"
    info_dir = dataset_dir + "info/"

    with open(dataset_dir + "/gen_settings.yaml", 'r') as sett_file:
        gen_sett = yaml.load(sett_file)

    def make_hash(obj):
        if isinstance(obj, (set, tuple, list)):
            return hash(tuple([make_hash(o) for o in obj]))
        elif isinstance(obj, dict):
            return make_hash([obj[k] for k in obj.keys()])
        elif isinstance(obj, np.ndarray):
            return hash(str(obj)) # imperfect; string does not show all data
        else:
            print(obj)
            return hash(obj)
    
    dataset_hash = make_hash(gen_sett) + hash(PREPROCESS_VERSION)
    dataset_pickle = dataset_dir + str(dataset_hash) + ".pk"
    if os.path.exists(dataset_pickle):
        print("Loading existing dataset: " + sub_dir)
        with open(dataset_pickle, 'r') as file:
            return pk.load(file)
    print("Loading dataset from files in: " + sub_dir)

    dataset = []

    info_files = os.listdir(info_dir)
    
    i = 0
    for f in info_files:
        i = i + 1
        print("Loading file %d of %d" % (i, len(info_files)))
        if not f.endswith('.yaml'):
            continue
        f = f.split('.yaml')[0]
        
        image_file = data_dir + f + ".png"
        info_file = info_dir + f + ".yaml"

        image = cv2.imread(image_file, cv2.IMREAD_UNCHANGED)

        with open(info_file, 'r') as file:
            info = yaml.load(file, Loader=yaml.SafeLoader)

        orig_sample = preprocess_data(image, info)
        #cv2.imshow('orig', orig_sample[0] + 0.5)
        #cv2.waitKey(0)

        for flip in [False, False]:
            for intensity in [1.0, 0.75, 0.5]:
                sample = orig_sample

                sample[2]['flipped'] = flip
                if flip:
                    sample = (np.flip(sample[0],1), -sample[1], sample[2])

                sample[2]['intensity'] = intensity
                sample = (intensity * (sample[0] + 0.5) - 0.5, sample[1], sample[2])
                
                dataset.append(sample)
                # cv2.imshow('image - %s - %s' % (str(flip), str(intensity)),sample[0] + 0.5)
                # print(sample[1])
                # cv2.waitKey(0)

    print("Saving dataset to %s" % dataset_pickle)
    with open(dataset_pickle, 'w') as file:
        pk.dump(dataset, file)
    return dataset


def load_datasets(pkg_dir, settings):
    dataset = []

    for sub_dir in settings['sub_dirs']:
        dataset = dataset + load_dataset(pkg_dir, settings, sub_dir)

    return dataset

def nvidiaBaseModel():
    model = Sequential()
    
    model.add(Conv2D(3, (5,5), strides=(2,2), input_shape= (OUT_HEIGHT,OUT_WIDTH,1), activation='elu'))
    
    model.add(Conv2D(24, (5,5), strides=(2,2), activation='elu'))
    model.add(Dropout(0.2))
    
    model.add(Conv2D(36, (5,5), strides=(2,2), activation='elu'))
    
    model.add(Conv2D(48, (3,3), strides=(2,2), activation='elu'))
    model.add(Dropout(0.3))
    
    model.add(Conv2D(64, (3,3), strides=(2,2), activation='elu'))
    
    model.add(Flatten())
    model.add(Dropout(0.4))
    
    model.add(Dense(1164, activation='elu'))
    model.add(Dense(100, activation='elu'))
    model.add(Dense(50, activation='elu'))
    model.add(Dense(10, activation='elu'))
    model.add(Dropout(0.5))
    
    model.add(Dense(1))
    
    optimizer = Adam(learning_rate=1e-5)
    model.compile(loss='mse', optimizer=optimizer, metrics=['mse', rmse, r_square])
    
    print(model)
    return model

# root mean squared error (rmse) for regression
def rmse(y_true, y_pred):
    from keras import backend
    return backend.sqrt(backend.mean(backend.square(y_pred - y_true), axis=-1))

# coefficient of determination (R^2) for regression
def r_square(y_true, y_pred):
    from keras import backend as K
    SS_res =  K.sum(K.square(y_true - y_pred)) 
    SS_tot = K.sum(K.square(y_true - K.mean(y_true))) 
    return (1 - SS_res/(SS_tot + K.epsilon()))

def plot_results(result, y_test, y_pred):
    #https://github.com/keras-team/keras/issues/7947
    #-----------------------------------------------------------------------------
    # Plot learning curves including R^2 and RMSE
    #-----------------------------------------------------------------------------

    # plot training curve for R^2 (beware of scale, starts very low negative)
    plt.plot(result.history['val_r_square'])
    plt.plot(result.history['r_square'])
    plt.title('model R^2')
    plt.ylabel('R^2')
    plt.xlabel('epoch')
    plt.legend(['train', 'test'], loc='upper left')
    plt.show()
            
    # plot training curve for rmse
    plt.plot(result.history['rmse'])
    plt.plot(result.history['val_rmse'])
    plt.title('rmse')
    plt.ylabel('rmse')
    plt.xlabel('epoch')
    plt.legend(['train', 'test'], loc='upper left')
    plt.show()


    plt.plot(result.history['loss'],color='blue')
    plt.plot(result.history['val_loss'],color='red')
    plt.title('loss')
    plt.ylabel('loss')
    plt.xlabel('epoch')
    plt.legend(["training loss", "validation loss"])
    plt.show()

    # print the linear regression and display datapoints
    from sklearn.linear_model import LinearRegression  
    regressor = LinearRegression()  
    regressor.fit(y_test.reshape(-1,1), y_pred)  
    y_fit = regressor.predict(y_pred) 

    reg_intercept = round(regressor.intercept_[0],4)
    reg_coef = round(regressor.coef_.flatten()[0],4)
    reg_label = "y = " + str(reg_intercept) + "*x +" + str(reg_coef)

    plt.scatter(y_test, y_pred, color='blue', label= 'data')
    plt.plot(y_pred, y_fit, color='red', linewidth=2, label = 'Linear regression\n'+reg_label) 
    plt.title('Linear Regression')
    plt.legend()
    plt.xlabel('observed')
    plt.ylabel('predicted')
    plt.show()

    #-----------------------------------------------------------------------------
    # print statistical figures of merit
    #-----------------------------------------------------------------------------

    import sklearn.metrics, math
    print("\n")
    print("Mean absolute error (MAE):      %f" % sklearn.metrics.mean_absolute_error(y_test,y_pred))
    print("Mean squared error (MSE):       %f" % sklearn.metrics.mean_squared_error(y_test,y_pred))
    print("Root mean squared error (RMSE): %f" % math.sqrt(sklearn.metrics.mean_squared_error(y_test,y_pred)))
    print("R square (R^2):                 %f" % sklearn.metrics.r2_score(y_test,y_pred))

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    package_dir = rospack.get_path("road_dataset_generation")

    with open(package_dir + "/config/test_learning_config.yaml", 'r') as file:
            settings = yaml.load(file, Loader=yaml.SafeLoader)

    model_name = package_dir + '/output/models/' + settings['model_name'] + '.model'

    dataset = load_datasets(package_dir, settings)
    print("Loaded dataset of %d files" % len(dataset))

    train_set, test_set = sklearn.model_selection.train_test_split(dataset, test_size=0.2, random_state=0)
    test_set, validation_set = sklearn.model_selection.train_test_split(test_set, test_size=0.5, random_state=0)

    X_train, Y_train, D_train = arrange_data(train_set, "Training")
    X_val, Y_val, D_val = arrange_data(validation_set, "Validation")
    X_test, Y_test, D_test = arrange_data(test_set, "Test")

    es = EarlyStopping(monitor='val_loss', mode='min', verbose=1, patience=int(settings['epochs']/20))
    mc = ModelCheckpoint(model_name, monitor='val_loss', mode='min', verbose='1')

    if os.path.exists(model_name) and settings['load_latest']:
        model = keras.models.load_model(model_name, custom_objects={"rmse": rmse, "r_square": r_square})

        sample_to_test = 2
        X = X_train[sample_to_test]
        X = X.reshape([-1, OUT_HEIGHT, OUT_WIDTH, 1])
        Y = Y_train[sample_to_test]
        Yp = model.predict([X])
        print("Test Network")
        print(Y)
        print(Yp)
        print(Y-Yp)
        if not settings['train_existing']:
            exit()
    else:
        model = nvidiaBaseModel()

    print(model.summary())
    history = model.fit(X_train, Y_train,
                        epochs=settings['epochs'],
                        batch_size=8,
                        validation_data=(X_val, Y_val),
                        callbacks=[es, mc],
                        verbose=1)
    
    y_predictions = model.predict(X_test)

    plot_results(history, Y_test, y_predictions)
    #print(model.predict(X_train))

    #model.save(model_name)
