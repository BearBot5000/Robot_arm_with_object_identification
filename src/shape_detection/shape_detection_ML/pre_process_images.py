#preprocessing for shape images to train for a CNN

import tensorflow
import os
import cv2
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from keras.utils import to_categorical
from sklearn.preprocessing import LabelEncoder

dataset_path = 'replace with actual path'
train_file = os.path.join(dataset_path, 'train')
test_file = os.path.join(dataset_path, 'test')


def no_of_subdirs(directory, set_name):
    counts = {}
    for item in os.listdir(directory):
        item_path = os.path.join(directory, item)
        if os.path.isdir(item_path):
            counts[item] = len(os.listdir(item_path))
    df = pd.DataFrame(counts, index=[set_name])
    return df

train_count = no_of_subdirs(train_file, 'train')
test_count = no_of_subdirs(test_file, 'test')
print(train_count)
print(test_count)

img_size = (48, 48)

train_images = []
train_labels = []
test_images = []
test_labels = []

def process_dataset(dataset_dir):
    images = []
    labels = []

    for emotion_dir in os.listdir(dataset_dir):
        emotion_path = os.path.join(dataset_dir, emotion_dir)
        if not os.path.isdir(emotion_path):
            continue
        for image_file in os.listdir(emotion_path):
            image_path = os.path.join(emotion_path, image_file)
            #load image
            image = cv2.imread(image_path)
            #resize and grayscale image
            resized_image = cv2.resize(image, img_size)
            grayscale_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
            images.append(grayscale_image)
            labels.append(emotion_dir)

    return images, labels

# Process training dataset
train_images, train_labels = process_dataset(train_file)

# Process testing dataset
test_images, test_labels = process_dataset(test_file)

# %%
def preprocess_dataset(images, labels):
    #convert to numpy array
    images = np.array(images)

    #convert labels to integer
    label_encoder = LabelEncoder()
    integer_labels = label_encoder.fit_transform(labels)

    #one-hot encoding labels
    num_classes = len(np.unique(integer_labels))
    onehot_labels = to_categorical(integer_labels, num_classes)

    #pixel normalization
    images = images / 255.0

    return images, onehot_labels, label_encoder

train_images, train_labels, train_label_encoder = preprocess_dataset(train_images, train_labels)
test_images, test_labels, test_label_encoder = preprocess_dataset(test_images, test_labels)

print("Train Images Shape:", train_images.shape)
print("Train Labels Shape:", train_labels.shape)
print("Test Images Shape:", test_images.shape)
print("Test Labels Shape:", test_labels.shape)

train_emotion_labels = train_label_encoder.classes_
test_emotion_labels = test_label_encoder.classes_

print("\nTrain Emotion Labels:")
for emotion, label in zip(train_emotion_labels, range(len(train_emotion_labels))):
    print(f"{emotion} {label}")

print("\nTest Emotion Labels:")
for emotion, label in zip(test_emotion_labels, range(len(test_emotion_labels))):
    print(f"{emotion} {label}")


np.save('train_images.npy', train_images)
np.save('train_labels.npy', train_labels)
files.download('train_images.npy')
files.download('train_labels.npy')

np.save('test_images.npy', test_images)
np.save('test_labels.npy', test_labels)
files.download('test_images.npy')
files.download('test_labels.npy')


