# Toyota Software Innovation Challenge – Vehicle Automation

## Software Challenge Event Schedule
|Friday, May 23| Saturday, May 24 | Sunday, May 25 |
| --- | --- | --- |
| 6:15-8pm: Workshop on coding practice and setup | 9am: Challenge introduction from Toyota <br> 12pm: Lunch <br> 5pm: Work period ends| ??? |


## Background

TMMC has several factories that build vehicles using parts with complex and intricate shapes. TMMC is developing a new solution to automate and speed up the movement of these parts using automated guided vehicles (AGVs). These vehicles will be able to pick up and automatically relocate specific parts between storage units in different areas of the facility without colliding with walls or other vehicles. 


TMMC has several factories that build vehicles using parts with complex and intricate shapes. TMMC is developing a new solution to automate and speed up the movement of these parts using automated guided vehicles (AGVs). These vehicles will be able to pick up and automatically relocate specific parts between storage units in different areas of the facility without colliding with walls or other vehicles.

![image from https://intranav.com](https://github.com/IdeasClinicUWaterloo/TIC_Software_Challenge_S25/blob/main/images/AGV_art.png)


## Learning Objectives

- Give students lower-stakes exposure to designing automated software systems
- Teach the importance of iterative design and careful planning
- Introduce and encourage research into technology and systems used in the workplace

## Challenge Description

**Problem:**

The focus of the TMMC Engineering project team is to find a reliable solution for using Automated Guided Vehicles (AGVs) to pick up and deliver vehicle components to defined stations in the manufacturing environment. Your task for the software section of this challenge is to program the movement of these vehicles.

### Requirements:

Your AGV is required to:

- Deliver an item from pickup point to delivery point, while:
  - Avoiding collisions with static fixtures (i.e. walls), and other AGVs
  - Obeying traffic signals (i.e. stop signs)

The ideal AGV will minimize the time of delivery.

### Recommended Breakdown:
To assist with this challenge, we have created a set of standardized validation tests of increasing difficulty. These tests are:

| **Challenge** | **Breakdown** |
|------------|----------------------------|
|**Level 1:** keyboard control with safety features| - Collision detection with wall <br> - Robot should stop and back up before it would collide
|**Level 2:** keyboard control with awareness|- Stop sign detection <br>- Robot should obey standard road rules when reaching a stop sign <br> - Describe and demonstrate your approach
|**Level 3:** autonomous control and static obstacles|- Can go through one loop of the track without keyboard control <br>- Collision and stop sign detection <br>- Record fastest delivery time
|**Level 4:** autonomous control and dynamic obstacles|- Moving robot obstacle (NPC) <br>- Describe and demonstrate your strategy <br>- Only one AGV <br>- Record fastest delivery time
|**Level 5:** multi agent/leet control|- 2 AGVs + moving robot (NPC)


# Test Setup

![Test room map](https://github.com/IdeasClinicUWaterloo/TIC_Software_Challenge_S25/blob/main/images/test_setup_1.png)


When you are ready to test on either a partial, or full course, you need to sign up for a testing timeslot [online]() [put link here]. To ensure fair access to the limited number of test setups, **these time slots are 20 minutes long, and one group cannot book two adjoining time slots**. Full test courses are in E7-1437, and partial test courses are in E7-1427 (see map above).

**Note: Validation steps must be completed in the order given in the table above.**![A maze made out of cardboard

![Testing environment setup](https://github.com/IdeasClinicUWaterloo/TIC_Software_Challenge_S25/blob/main/images/test_setup_2.png)

## Final demonstration and presentation

The final presentations will take place Sunday afternoon to event organizers and TMMC staff. Please take note of the following:

- Your presentation should include a recording of your robot. This could be either a video recording of the physical robot, or a screen capture in simulation. Show the highest-level functionality that you have completed.
- Your presentation must use the TMMC Powerpoint template, found online \[put link here\]
- You must sign up for your **7-minute** presentation time online \[put link here\]. Put your name under one of the three judges in one of the allotted timeslots. Please leave enough time for the next group to set up.

![Judging room map](https://github.com/IdeasClinicUWaterloo/TIC_Software_Challenge_S25/blob/main/images/test_setup_3.png)


## Software Resources

- To set up the environment to do this challenge, you will need about 50GB of space. Instructions for setting up and connecting to the TurtleBot can be found here: ![](data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAABGdBTUEAALGPC/xhBQAAAEdJREFUOE9joBhYWVn8x4VdXZ02QpXhBiCFv379wsBXr17Faig6xuuC2tpqrIbDMEgNThcQg0cNGDUAhKlnACUYmqXIBQwMAGOVwgW7hjTIAAAAAElFTkSuQmCC)[Virtual Machine Resources and Documentation](https://uofwaterloo.sharepoint.com/:f:/s/tm-eng-engineeringideasclinic/EqQT8roBWSdEt7gf6-fqh_EB9GdGZYsEpoARi1OnbbB0uw?e=pfKMXH).
- The functions have been written to streamline your development. The following documentation describes those functions: [Robot Documentation](https://docs.google.com/document/d/1OMNe_YqdWdEfYw3CDldJkrOWBQl-ZO1z/edit)