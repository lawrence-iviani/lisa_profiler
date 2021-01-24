# lisa_profiler
a collection of scripts for profiling LISA leveraging the ROS middleware bus


Two ROS nodes are provided, one for running the test player, reproducing from a json file in the form **(TODO: explain the json format for test set)**
The first play the file and instruct to creat a bag file. The bag file has to be processed to extract csv files consumable by the scripts in 
data_processing_pycharm/verification_evaluation.py

```batch
$ rosrun lisa_profiler player.py  -b 5000 -q 50  -r  /home/osboxes/fzi_dependecies/robot_folders/checkout/lisa_test/catkin_ws/src/lisa_profiler/test_resources/descriptions/testset_20201222_short_v1.json 
$ rosrun lisa_profiler process_bag.py -b Respeaker_spk80db_quiet_room.bag 
```

Below the help options for both

```batch
$ rosrun lisa_profiler player.py -h

usage: player.py [-h] [-l] [-d DEVICE] [-b BLOCKSIZE] [-q BUFFERSIZE] [-r]
                 FILENAME

Play an audio file using a limited amount of memory.

positional arguments:
  FILENAME              JSON audio description of the playback

optional arguments:
  -h, --help            show this help message and exit
  -l, --list-devices    show list of audio devices and exit
  -d DEVICE, --device DEVICE
                        output device (numeric ID or substring)
  -b BLOCKSIZE, --blocksize BLOCKSIZE
                        block size (default: 2048)
  -q BUFFERSIZE, --buffersize BUFFERSIZE
                        number of blocks used for buffering (default: 20)
  -r, --rosbag          save a rosbag file in the actual folder (default:
                        False)
```



	
```batch						
$ rosrun lisa_profiler process_bag.py -h

usage: process_bag.py [-h] -b BAG [-k [KEY [KEY ...]]] [-o] [-v]

Print one or multiple bag keys

optional arguments:
  -h, --help            show this help message and exit
  -b BAG, --bag BAG     Bag file to read
  -k [KEY [KEY ...]], --key [KEY [KEY ...]]
                        Key you would like to print
  -o, --output          Output in CSV
  -v, --verbose         Log verbose
osboxes@osboxes:~$ 

```