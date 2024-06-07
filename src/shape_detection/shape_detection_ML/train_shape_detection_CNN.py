#train a Convolution Neural Network on shape images
#need test images


import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout, BatchNormalization, GlobalAveragePooling2D
from tensorflow.keras.preprocessing.image import ImageDataGenerator

dataset_path = 'replace with data path'

#ensure two folders with preprocessed training and test images of shapes
train_images = np.load(dataset_path + 'train_images.npy')
train_labels = np.load(dataset_path + 'train_labels.npy')
test_images = np.load(dataset_path + 'test_images.npy')
test_labels = np.load(dataset_path + 'test_labels.npy')

print("Train Images Shape:", train_images.shape)
print("Train Labels Shape:", train_labels.shape)
print("Test Images Shape:", test_images.shape)
print("Test Labels Shape:", test_labels.shape)

print("\nTrain Images Data Type:", train_images.dtype)
print("Train Labels Data Type:", train_labels.dtype)
print("Test Images Data Type:", test_images.dtype)
print("Test Labels Data Type:", test_labels.dtype)

print("\nTrain Images Min Value:", np.min(train_images))
print("Train Images Max Value:", np.max(train_images))
print("Test Images Min Value:", np.min(test_images))
print("Test Images Max Value:", np.max(test_images))

print("\nUnique Train Labels:", np.unique(train_labels))
print("Unique Test Labels:", np.unique(test_labels))



input_shape = (48, 48, 1)

model = Sequential([
    Conv2D(32, (3, 3), activation='relu', padding='same', input_shape=input_shape),
    BatchNormalization(),
    Conv2D(32, (3, 3), activation='relu', padding='same'),
    BatchNormalization(),
    MaxPooling2D(2, 2),

    Conv2D(64, (3, 3), activation='relu', padding='same'),
    BatchNormalization(),
    Conv2D(64, (3, 3), activation='relu', padding='same'),
    BatchNormalization(),
    MaxPooling2D(2, 2),

    Conv2D(128, (3, 3), activation='relu', padding='same'),
    BatchNormalization(),
    Conv2D(128, (3, 3), activation='relu', padding='same'),
    BatchNormalization(),
    MaxPooling2D(2, 2),

    Conv2D(256, (3, 3), activation='relu', padding='same'),
    BatchNormalization(),
    Conv2D(256, (3, 3), activation='relu', padding='same'),
    BatchNormalization(),
    GlobalAveragePooling2D(),

    Dense(256, activation='relu'),
    BatchNormalization(),
    Dropout(0.3),
    Dense(7, activation='softmax')
])

model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

model.summary()

epochs = 40
batch_size = 32

#Data augmentation parameters
datagen = ImageDataGenerator(
    rotation_range=20,
    width_shift_range=0.2,
    height_shift_range=0.2,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=True,
    fill_mode='nearest'
)

train_images = train_images.reshape((-1, 48, 48, 1))
test_images = test_images.reshape((-1, 48, 48, 1))

train_generator = datagen.flow(
    train_images,
    train_labels,
    batch_size=batch_size
)

model.fit(
    train_generator,
    steps_per_epoch=len(train_images) // batch_size,
    epochs=epochs,
    validation_data=(test_images, test_labels)
)

loss, accuracy = model.evaluate(test_images, test_labels)
print(f"Test Loss: {loss:.4f}")
print(f"Test Accuracy: {accuracy:.4f}")

#save a large scale model
model.save('emotion_detection_model.h5')
#convert and save to a tflite model to preserve computational power
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

with open('emotion_detection_model.tflite', 'wb') as f:
    f.write(tflite_model)

files.download('emotion_detection_model.h5')
files.download('emotion_detection_model.tflite')