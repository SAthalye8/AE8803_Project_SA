# AE8803 Project Code and Results (Spring 2023)
## Centralized Approaches
- Safe-interval Path Planning (SIPP):

![sipp](https://user-images.githubusercontent.com/103329531/235287490-491df5c4-34f3-4ea6-aab1-b054ae5956c0.gif)


- Conflict-based Search (CBS):

![cbs](https://user-images.githubusercontent.com/103329531/235287498-cb8a709f-1319-4921-9af6-c88278a985e7.gif)

- Comparison of SIPP (failure) and CBS (success) for 3-agent network:

SIPP             |  CBS
:-------------------------:|:-------------------------:
![sipp_3_failure](https://user-images.githubusercontent.com/103329531/235287913-30edaa53-3ae9-4f34-97c1-5a9f2282a6cd.gif)  |![cbs_3](https://user-images.githubusercontent.com/103329531/235287920-d76bbde4-fa3f-4b4b-a4b5-cab38970ac2c.gif)

 ### *To reproduce the above results:* 
  - Unzip the `Centralized_methods.zip` folder
  - Install the necessary dependencies:
  ```
  python3 -m pip install -r requirements.txt
  ```
  - For SIPP results, run:
  ```
  cd ./sipp
  python3 multi_sipp.py input.yaml output.yaml
  python3 visualize_sipp.py input.yaml output.yaml 
  ```
   - For CBS results, run:
  ```
  cd ./cbs
  python3 cbs.py input.yaml output.yaml 
  python3 ../visualize.py input.yaml output.yaml
  ```  
  
  - Code adapted from: https://github.com/atb033/multi_agent_path_planning
  
  
## Decentralized Approaches
 - Velocity obstacle (VO):
 ![VO](https://user-images.githubusercontent.com/103329531/206857905-3a6b6442-da06-4254-8b1d-a8de3f5e57ca.gif)

 - Hybrid reciprocal velocity obstacle (HRVO): 
 ![HRVO](https://user-images.githubusercontent.com/103329531/206858041-9f0598bd-eee4-4a97-90ef-94e8a6bf3929.gif)
 
 ### *To reproduce the above results:* 
  - Install the necessary dependencies:
  ```
  python3 -m pip install -r requirements.txt
  ```
  - Run the code:
  ```
  python3 AE8803_Decentralized.py
  ```
  - Code adapted from: https://github.com/MengGuo/RVO_Py_MAS
