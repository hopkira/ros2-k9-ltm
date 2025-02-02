# ROS2 Long Term Memory Service

This repository contains a ROS2 retrival augmented generation (RAG) service for managing long term memory in the form of a service and a test client. The service provides capabilities to choose between ollama models, query the robot's memory, and remember new facts. This README file will guide you through deploying and running this service in a ROS2 environment.

## Overview

The Long Term Memory Service is designed to allow users to interact with a system that can store and retrieve information over time. The information is unstructured and typically biographical. The service will load a backstory for K9 that was generated from Wikipedia synopses of the stories he was in (from a CSV file), but additional facts can be permamently added to his long term memory using the remember capability.

The service uses two types of models: a generative model (GEN_MODEL) and an embedding model (EMBED_MODEL). These two models support the Retrieval Augmented Generation (RAG) pattern that is at the heart of this implementation. The service offers three primary functionalities: changing the ollama models, querying the memory for facts, and remembering new facts.

## Deployment and Running

### Prerequisites
- Raspberry Pi 5 (or better)
- Ubuntu Linux - Noble (24.04) 64-bit
- ROS2 Jazzy Jalisco
- Ollama
- Python3
- pip (Python package installer)
- git

### Installation
1. Install [Ollama|https://ollama.com/download] and install the base models:
```bash
ollama pull granite-embedding:30m
ollama pull granite3-moe:3b
```
2. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/ROS2-LTM.git
   ```
3. Navigate to the cloned directory and install the Python dependencies:
   ```bash
   pip install ollama chromadb pandas --break-system-packages --ignore-installed
   ```

### Running the Service
1. Source your ROS2 environment if not already sourced:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```
2. Create the package (from the ROS2-LTM director)
   ```bash
   ros2 pkg create --build-type ament_python --license Apache-2.0 ltm_package --dependencies service_ltm rclpy std_msgs example_interfaces ltm_test_client
   ```
3. Build the package:
   ```bash
   cd ltm_package
   colcon build
   ```
4. Source the setup files:
   ```bash
   . install/setup.bash
   ```

5. Run the service server:
   ```bash
   ros2 run ltm_package service_ltm
   ```
6. In a new terminal, test the service by running the client:
   ```bash
   ros2 run ltm_package ltm_test_client
   ```

## Exercising Service Capabilities from Command Line
You can exercise the service capabilities directly from the command line using ROS2 commands. Here are examples:
- Setting a model:
  ```bash
  ros2 service call /ltm_set_model ltm_package/LtmSetModel "{model_type: EMBED_MODEL, model_name: 'nomic-embed-text'}"
  ```
- Querying memory:
  ```bash
  ros2 service call /ltm_query ltm_package/LtmQuery "query: 'What happened to the parrot?'"
  ```
- Remembering up to twenty new topics and related facts:
  ```bash
  ros2 service call /ltm_remember ltm_package/LtmRemember "up_to_twenty_topics_of_512_chars: ['your_topic','your_topic_2', 'your_topic_n']"
  ```

## File Descriptions

| File Name            | Description                                               |
|----------------------|-----------------------------------------------------------|
| service_ltm.py       | Python script containing the service node implementation  |
| ltm_set_model.srv    | Service definition file for changing the ollama model              |
| ltm_query.srv        | Service definition file for querying memory               |
| ltm_remember.srv     | Service definition file for remembering new facts         |
| test_ltm.py          | Python script containing the service test client          |
| package.xml          | XML file describing your ROS2 package                     |
| CMakeLists.txt       | CMake file to build your ROS2 package                     |
| k9_stories_500.csv   | Basic facts harvested from Wikipedia (see who_uni repo)   |