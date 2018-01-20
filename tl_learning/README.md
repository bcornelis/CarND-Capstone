## Explanation

This directory contains files to setup and run the [TensforFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection).

### Dataset
The dataset used, dataset-sdcnd-capstone.zip, is not my own work and has been downloaded from the Udacity forums.

### Files
This directory contains the following files:
* [object_detection_test.ipynb](object_detection_test.ipynb): This file is copied from the original [Object Detection API tutorial](https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb) and has been slightly modified to test the model for the current structure
* [rfcn_resnet101_coco-udacity_sim.config](rfcn_resnet101_coco-udacity_sim.config): this file is copied from the original [Object Detection API config file](https://github.com/tensorflow/models/blob/master/research/object_detection/samples/configs/rfcn_resnet101_coco.config) and modified to process the correct number of classes (4 instead of 90) and link to the correct model/labels/... files
* [label_map.pbtxt](label_map.pbtxt): included in the dataset

### Process

#### Training

To train the model, perform the following steps in the current directory:

1. Clone the TensorFlow Object Detection API
```bash
git clone https://github.com/tensorflow/models
cd models/research
protoc object_detection/protos/*.proto --python_out=.
cd ..
cd ..

```
There is a problem with this version while exporting the inference graph, so [this](https://github.com/tensorflow/models/issues/2861) fix has to be applied
```bash
sed -i -e 's/layout_optimizer/optimize_tensor_layout/g' models/research/object_detection/exporter.py
```

2. Download the models to test with
```bash
mkdir pre_trained
cd pre_trained
for model in \
  ssd_mobilenet_v1_coco_11_06_2017 \
  ssd_inception_v2_coco_11_06_2017 \
  rfcn_resnet101_coco_11_06_2017 \
  faster_rcnn_resnet101_coco_11_06_2017 \
  faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017
do \
  curl -OL http://download.tensorflow.org/models/object_detection/$model.tar.gz
  tar -xzf $model.tar.gz 
done
rm *.tar.gz
cd ..
```

3. Download the annotated data. To train the model, the data has to be available and annotated. Annotated means that the original images should be annotated with rectangles and a label so TensorFlow can learn from this data. Luckily the annotated Udacity simulator/real data is already available:
```bash
mkdir dataset
wget ...
unzip dataset-sdcnd-capstone.zip -d dataset
rm dataset-sdcnd-capstone.zip

```
4. Set the PYTHONPATH variable
```bash
export PYTHONPATH=/home/bcornelis/UdacitySelfDrivingCar/term3/CarND-Capstone/tl_learning/models/research/:/home/bcornelis/UdacitySelfDrivingCar/term3/CarND-Capstone/tl_learning/models/research/slim
```

4. Train the model
```bash
python models/research/object_detection/train.py \
  --pipeline_config_path=./rfcn_resnet101_coco-udacity_sim.config \
  --train_dir=dataset/data/sim_training_data/sim_data_capture/
```

5. Export for inference (replace the 233 in the command to the latest version available)
```bash
python models/research/object_detection/export_inference_graph.py \
  --pipeline_config_path=./rfcn_resnet101_coco-udacity_sim.config \
  --trained_checkpoint_prefix=dataset/data/sim_training_data/sim_data_capture/model.ckpt-233 \
  --output_directory=frozen/
```

#### Testing
For testing, just open the Jupyter notebook and execute all the steps:
```bash
jupyter notebook object_detection_test.ipynb
```
