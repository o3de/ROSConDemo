# Apple Tree Functional Requirements

## Goal
The goal for the Apple Tree environment is to simulate realistic physical behavior of simulated apples on apple trees when they are picked by a robotic device (i.e. arm). 


### Apple Tree Setup/Configuration
* For phase one, each apple in the apple tree prefab will be its one individual prefab
* The apples will be randomly placed as design time for the level
* The apple trees will be manually layed out on the apple orchard 

### Handling picking apples

* Apple Prefabs are pre-attached to the Apple Tree Prefab. (No dynamic random spawning)
* Apple Tree Prefabs will be instantiated in a set/fixed row of apple trees
* The Apple Prefab entity will add a PhysX rigid body, with gravity initially disabled
* During runtime of the level, the robotic arm will define a volume box to query for entities that fit inside the box
    * The volume box to query must be slightly larger than the apples
* The robot script will act upon any apple that triggers the volume box query
* The robot script will detach the apple through the manipulator and collect it into a basket
  

