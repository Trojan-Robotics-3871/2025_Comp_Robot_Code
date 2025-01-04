# Reefscape!
This code is for the Reefscape season and is currently unfinished.

# Trojans Robotics - 3871
This is the robotics code used during the Reefscape Regionals competition in Minneapolis in 2025. This copy of the code includes a document that has instructions on how to configure the program for the robot, upload it, etc.

# Setting up laptop for competition

1. Make sure you disable the internet on the laptop

Search - View internet connections
- Disable Wi-Fi

2. Disable firewall protections on the laptop

Search - Firewall & Network Protection
- Advanced Settings
- Windows Defender Firewall State
- Disable ALL Firewalls
- Apply Settings


# Documentation for newbies
If you're new to our robotics team, welcome! We're glad you joined us. If you're interested in joining the cool kids who do the programming for the robot, we have some tools you can use to get started.

Basic information for getting started on programming can be found on the [official FRC website](https://docs.wpilib.org/en/stable/index.html), which will teach you how to use Java and getting to know our code.

If you have a question on something that the team isn't able to help you on, there are many people who may be able to help you. You can start by requesting for help on the [Chief Delphi](https://www.chiefdelphi.com/) website. **You will have to create an account if you wish to do so.**

# Installing required libraries
If you're installing the FRC programs and the robotics code on a new computer, you will have to install the required libraries for the robot to function properly. 

For the year 2024, there's two libraries we use primarily in the code. We use the **Revrobotics** and the **PlayingWithFusion** libraries, which have to be installed separately.

The links to these packages are as follows:
https://software-metadata.revrobotics.com/REVLib-2025.json
https://www.playingwithfusion.com/frc/playingwithfusion2024.json

To install these packages, you can do the following:

- Press the WPILib button in Visual Studio Code, and search for **Manage Vendor Libraries**
- Press **Install new libraries (Online)**
- Paste one of the links into the field, and then press enter. WPILib will ask to build the project and then restart the app, allow it to do so.

> **IMPORTANT: Please note that the Revrobotics library will NOT install on the school internet as the link is blocked. Recommendation is to use a hotspot to add it to your computer.**

If there are errors in your code after the installation, you may have to run the project configuration tool which will allow the project to recognize the libraries. **Contact Brayden if you need help doing this step.**

# Formatting the code
The code needs to stay properly formatted for remaining clean and readable by all contributors. If you wish to format the code yourself, you will need to press the keys **Ctrl+K and Ctrl+F** one at a time.

# Uploading the code
Before uploading the code to the robot, ensure that there are no errors on the code. The robot will **NOT** build successfully if there are active errors, so ensure stability before uploading.

To upload the code, you may do the following:
 - Press FN + Shift + F5 on your keyboard all at the same time
 - Alternatively, you can upload the code by pressing the WPILib button and searching for "Deploy Robot Code".

If you're continuing to have issues after uploading the code and there is no errors, there may be a silent error that is happening. Ensure there is no issues on the robot's hardware as well, and check that CAN wiring is all in place.

# Modifying drive-motor speed
The motors maximum speed is configured **1**, with the lowest possible being **0**. 

Consider the number 1 being 100% speed, and by lowering it to 0.5 you are halving the speed to 50%.

You can edit the speed by going into the code and editing the value that is attached to the **maxSpeed** configuration.

**Please note that this won't modify the speed for the REV motors, as this is a different configuration and needs to be changed differently.**

> **Only edit the speed if we're doing a parade or freshman orientation! The speed should NOT be modified if we are at competition.**
