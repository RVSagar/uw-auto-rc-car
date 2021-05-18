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

from keras.models import Sequential
from keras.layers import Conv2D
from keras.layers import MaxPooling2D
from keras.layers import Dense
from keras.layers import Flatten
from keras.layers import LeakyReLU
from keras.optimizers import SGD


def arrange_data(dataset, name):
    print("Arranging: %s" % name)

    X = [dat[0] for dat in dataset]
    Y = [dat[1] for dat in dataset]
    D = [dat[2] for dat in dataset]

    X = np.array(X)
    X = X.reshape([-1,60,80,1])

    Y = np.array(Y)
    Y = Y.reshape([-1, 3])
    print("  X: " + str(X.shape))
    print("  Y: " + str(Y.shape))
    return X, Y, D


def preprocess_data(X, D):
    grey = cv2.cvtColor(X, cv2.COLOR_BGR2GRAY)

    # HEIGHT = 480
    # WIDTH = 640
    small = cv2.resize(grey, (80, 60))

    Y = np.array([[D['X0']],
                  [D['YAW0']],
                  [D['inv_rad']]])
    return (small, Y, D)

def load_datasets(pkg_dir, settings):
    dataset = []

    sub_dir_hash = 0
    for sub_dir in settings['sub_dirs']:
        sub_dir_hash = sub_dir_hash + hash(sub_dir)
    dataset_pickle = pkg_dir + "/output/" + str(sub_dir_hash) + ".pk"
    
    if os.path.exists(dataset_pickle):
        print("Loading existing dataset")
        with open(dataset_pickle, 'r') as file:
            return pk.load(file)
    print("Loading dataset from files")

    for sub_dir in settings['sub_dirs']:
        dataset_dir = pkg_dir + "/output/" + sub_dir + "/"

        data_dir = dataset_dir + "data/"
        info_dir = dataset_dir + "info/"

        info_files = os.listdir(info_dir)

        for f in info_files:
            if not f.endswith('.yaml'):
                continue
            f = f.split('.yaml')[0]
            
            image_file = data_dir + f + ".png"
            info_file = info_dir + f + ".yaml"

            image = cv2.imread(image_file, cv2.IMREAD_UNCHANGED)

            with open(info_file, 'r') as file:
                info = yaml.load(file, Loader=yaml.SafeLoader)

            sample = preprocess_data(image, info)
            dataset.append(sample)

    print("Saving dataset to %s" % dataset_pickle)
    with open(dataset_pickle, 'w') as file:
        pk.dump(dataset, file)
    return dataset


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    package_dir = rospack.get_path("road_dataset_generation")

    with open(package_dir + "/config/test_learning_config.yaml", 'r') as file:
            settings = yaml.load(file, Loader=yaml.SafeLoader)

    dataset = load_datasets(package_dir, settings)
    print("Loaded dataset of %d files" % len(dataset))

    train_set, test_set = sklearn.model_selection.train_test_split(dataset, test_size=0.2, random_state=0)
    test_set, validation_set = sklearn.model_selection.train_test_split(test_set, test_size=0.5, random_state=0)

    X_train, Y_train, D_train = arrange_data(train_set, "Training")
    X_val, Y_val, D_val = arrange_data(validation_set, "Validation")
    X_test, Y_test, D_test = arrange_data(test_set, "Test")

    model = Sequential()
    model.add(Conv2D(32, (5, 5), input_shape=(60,80,1)))
    model.add(LeakyReLU(alpha=0.3))
    model.add(Conv2D(32, (5, 5)))
    model.add(LeakyReLU(alpha=0.3))
    model.add(MaxPooling2D((2, 2)))
    model.add(Conv2D(32, (5, 5)))
    model.add(LeakyReLU(alpha=0.3))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(LeakyReLU(alpha=0.3))
    model.add(Dense(3, activation=None))
    # compile model
    opt = SGD(lr=0.01, momentum=0.9, clipnorm=1)
    model.compile(optimizer=opt, loss='mse', metrics=['mse'])

    history = model.fit(X_train, Y_train, epochs=10, batch_size=32, validation_data=(X_val, Y_val), verbose=1)
    #print(model.predict(X_train))

    model.save(package_dir + '/output/models/' + settings['model_name'] + '.model')
