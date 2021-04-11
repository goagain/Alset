# Capstone Project: Automatic Driving by Taking Advantage of Carla

CARLA is an open-source simulator for autonomous driving research. It provides various functions like 3D environment simulation, spawning NPC, and having different kinds of sensor data ready. CARLA offers a great base we could build our automatic driving system.
There are three Switch modes in total that players can choose from:: manual operation, semi-automatic assistance, and fully automatic driving. Moreover, the player has an option to choose whether to create NPC and find their favorite vehicle to play. 
We create cameras to collect all images taken. Then, we have object detection function ready by taking davantage of YOLO.
In semi-automatic assitance mode, the suggested lane to drive will be shown in green. Also, the car will automatically slower its speed if it reaches speed limits.

## Reference
CARLA: An Open Urban Driving Simulator
Alexey Dosovitskiy, German Ros, Felipe Codevilla, Antonio Lopez, Vladlen Koltun; PMLR 78:1-16 [PDF] [talk]

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system. The recommended system is list below
- Intel i7 gen 9th - 11th / Intel i9 gen 9th - 11th / AMD ryzen 7 / AMD ryzen 9+16 GB RAM memory
- NVIDIA RTX 2070 / NVIDIA RTX 2080 / NVIDIA RTX 3070, NVIDIA RTX 3080
- Ubuntu 18.04

### Prerequisites

We need to build the CARLA first

```
#Building CARLA
# Download and install the UE patch  
cd ~/UnrealEngine_4.24
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/UE_Patch/430667-13636743-patch.txt ~/430667-13636743-patch.txt
patch --strip=4 < ~/430667-13636743-patch.txt
# Build UE
./Setup.sh && ./GenerateProjectFiles.sh && make
```

### Installing

A step by step series of examples that tell you how to get a development env running

Say what the step will be

```
Downloade the whole file and extract it
```

And 

```
Download Visual Studio Code
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Run main.py

### Break down into end to end tests

Explain what these tests test and why

```
Run Main for checking the whole UI
```


## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Calar](https://carla.org/) - The web framework used
* [UI](https://pypi.org/project/PyQt5/) - PyQt5

## Contributing

1. DataGenerateTool  
   You can generate and autolable image data by using autolabel_carla.py, create your own dataset in Carla.
   The name of the image and label will be same,  the label is Yolo formate txt file.
   
2. Dataset  
   We have generate a dataset,which can be trained in Yolo. You can download here.
   All data are labeled, which contains 17902 training data and 4548 validation data.
   
3. Pretained object detection model  
   object_detector.pt is the pretained model by Yolo v5 small, using the dataset generate from Carla.  You can using it to detect cars, pedestrian, traffic light(red, bule, yellow), speed sign(30, 60, 90) and bus station.



## Versioning

We have all files repsenting our developing steps. [Milestoen File](https://github.com/goagain/Alset-Autopilot-System/tree/data-collector/documents). 

## Authors

* **DC Capstone Group** - *Initial work* - [Calar](https://github.com/goagain/Alset-Autopilot-System)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under theApache License 2.0  - see the [LICENSE](LICENSE) file for details

## Acknowledgments

CARLA specific code is distributed under MIT License.
CARLA specific assets are distributed under CC-BY License.
The ad-rss-lib library compiled and linked by the RSS Integration build variant introduces LGPL-2.1-only License.
UE4 itself follows its own license terms.
