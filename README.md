# vigilant-enigmas

1. Launch the runway simulation in Gazebo  
` roslaunch iq_sim runway.launch --screen `

2. Launch ArduCopter  
` ~/startsitl.sh `

3. Launch mavros  
` roslaunch iq_sim apm.launch --screen `

4.Set Correct Cd

 `cd ~/GNC/src/gdp/scripts   `
 
5. Run Python file 

` python GNC_node_with_redundancy.py `
