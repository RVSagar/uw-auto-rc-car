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
from keras.optimizers import SGD
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
    Y = Y.reshape([-1, 3])
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
    

    Y = np.array([[D['X0']],
                  [D['YAW0']],
                  [D['inv_rad']]])
    
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

        for flip in [False, True]:
            for intensity in [1.0, 0.75, 0.5]:
                sample = orig_sample

                sample[2]['flipped'] = flip
                if flip:
                    sample = (np.flip(sample[0],1), -sample[1], sample[2])

                sample[2]['intensity'] = intensity
                sample = (intensity * (sample[0] + 0.5) - 0.5, sample[1], sample[2])
                
                dataset.append(sample)
                #cv2.imshow('image - %s - %s' % (str(flip), str(intensity)),sample[0] + 0.5)
                #print(sample[1])
                #cv2.waitKey(0)

    print("Saving dataset to %s" % dataset_pickle)
    with open(dataset_pickle, 'w') as file:
        pk.dump(dataset, file)
    return dataset


def load_datasets(pkg_dir, settings):
    dataset = []

    for sub_dir in settings['sub_dirs']:
        dataset = dataset + load_dataset(pkg_dir, settings, sub_dir)

    return dataset


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

    if os.path.exists(model) and settings['load_latest']:
        model = keras.models.load_model(model_name)

        X = X_train[0]
        X = X.reshape([-1, OUT_HEIGHT, OUT_WIDTH, 1])
        Y = Y_train[0]
        Yp = model.predict([X])
        print(Y)
        print(Yp)
        print(Y-Yp)
        if not settings['train_existing']:
            exit()
    else:
        model = Sequential()
        model.add(Conv2D(16, (5, 5), input_shape=(OUT_HEIGHT,OUT_WIDTH,1)))
        model.add(LeakyReLU(alpha=0.3))
        model.add(Dropout(0.4))
        model.add(Conv2D(32, (5, 5)))
        model.add(LeakyReLU(alpha=0.3))
        model.add(Dropout(0.4))
        model.add(MaxPooling2D((2, 2)))
        model.add(Conv2D(32, (5, 5)))
        model.add(LeakyReLU(alpha=0.3))
        model.add(Dropout(0.4))
        model.add(MaxPooling2D((2, 2)))
        model.add(Flatten())
        model.add(Dense(100))
        model.add(LeakyReLU(alpha=0.3))
        model.add(Dropout(0.4))
        model.add(Dense(3, activation=None))
        # compile model
        opt = SGD(lr=0.01, momentum=0.9, clipnorm=1)
        model.compile(optimizer=opt, loss='mse', metrics=['mse'])

    print(model.summary())
    history = model.fit(X_train, Y_train,
                        epochs=settings['epochs'],
                        batch_size=16,
                        validation_data=(X_val, Y_val),
                        callbacks=[es, mc],
                        verbose=1)
    #print(model.predict(X_train))

    #model.save(model_name)
