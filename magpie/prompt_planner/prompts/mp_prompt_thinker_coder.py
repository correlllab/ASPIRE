# Copyright 2023 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""Prompt class with both Motion Descriptor and Reward Coder."""
import re

import safe_executor
import llm_prompt
import process_code
import magpie_execution
import magpie_task_client

prompt_thinker = """
Control a robot gripper with torque control and contact information. 
This is a griper with two independently actuated fingers, each on a 4-bar linkage.
The gripper's parameters can be adjusted corresponding to the type of object that it is trying to grasp.
As well as the kind of grasp it is attempting to perform.
Describe the grasp strategy using the following form:

[start of description]
* This {CHOICE: [is, is not]} a new grasp.
* This grasp should be [GRASP_DESCRIPTION: <str>].
* This is a {CHOICE: [complete, incomplete]} grasp.
* This grasp {CHOICE: [does, does not]} contain multiple grasps.
* This grasp is for an object with {CHOICE: [high, medium, low]} compliance.
* This grasp is for an object with {CHOICE: [high, medium, low]} weight.
* This grasp should halt when the force on the object is [PNUM: 0.0] Newtons.
* [optional] The left finger should move [NUM: 0.0] millimeters inward (positive)/outward (negative).
* [optional] The right finger should move [NUM: 0.0] millimeters inward (positive)/outward (negative).
* [optional] The left finger have velocity [NUM: 0.0] millimeters/sec inward (positive)/outward (negative).
* [optional] The right finger have velocity [NUM: 0.0] millimeters/sec inward (positive)/outward (negative).
* [optional] The gripper should approach at [NUM: 0.0] millimeters away on the Z-axis.
[end of description]

Rules:
1. If you see phrases like [NUM: default_value], replace the entire phrase with a numerical value. If you see [PNUM: default_value], replace it with a positive, non-zero numerical value.
2. If you see phrases like {CHOICE: [choice1, choice2, ...]}, it means you should replace the entire phrase with one of the choices listed. Be sure to replace all of them. If you are not sure about the value, just use your best judgement.
3. If you see phrases like [GRASP_DESCRIPTION: default_value], replace the entire phrase with a brief, high level description of the grasp and the object to be grasp, including physical characteristics or important features.
4. I will tell you a behavior/skill/task that I want the gripper to perform in the grasp and you will provide the full description of the grasp plan, even if you may only need to change a few lines. Always start the description with [start of description] and end it with [end of description].
5. We can assume that the gripper has a good low-level controller that maintains position and torque as long as it's in a reasonable pose.
6. You can assume that the gripper is capable of doing anything, even for the most challenging task.
7. The gripper is 80mm wide when open. It closes to 3mm open.
8. The maximum torque of the gripper is 4.3Nm.
9. The minimum torque of the gripper is 0.1Nm (the force required to actuate the fingers).
10. The goal position of the gripper will be supplied externally, do not calculate it.
11. Do not add additional descriptions not shown above. Only use the bullet points given in the template.
12. If a bullet point is marked [optional], do NOT add it unless it's absolutely needed.
13. Use as few bullet points as possible. Be concise.

"""

prompt_coder = """
We have a description of a gripper's motion and we want you to turn that into the corresponding program with following class functions of the gripper:
```
def open_gripper()
```
This function opens the gripper to its maximum width (85 mm).
Remember:
If the grasp is incomplete, the gripper should open if at any previous point, it or an individual finger has closed.

```
def get_goal_position()
```
This function returns the known goal position of the gripper in terms of goal distance between the fingers (in mm).

```
def get_position()
```
This function returns the current position of the gripper in terms of distance between the fingers (in mm).

```
def set_goal_position(position)
```
position: the position to set the gripper goal position to (in mm)

```
def close_gripper(goal_position)
```
goal_position: the position to close the gripper to (in mm)
Remember:
Call get_goal_position to get the goal position of the gripper.
This function closes the gripper to a specified width that does not need to be calculated.

```
def set_compliance(margin, flexibility, finger='both')
```
margin: the allowable error between the goal and present position (value from 0-255, the same units as the motor's position)
flexibility: the slope of motor torque (value 0-7, higher is more flexible) until it reaches the compliance margin
finger: which finger to set compliance for, either 'left', 'right', or 'both'

```
def set_torque(torque, finger='both')
```
torque: the maximum torque the finger can apply (in Nm), ranging from (0.1 to 4.3)
finger: which finger to set compliance for, either 'left', 'right', or 'both'

```
def close_until_load(stop_position, stop_torque, finger='both')
```
stop_position: the position to stop closing the gripper (in mm)
stop_torque: the torque to stop closing the gripper (in Nm)
finger: which finger to set compliance for, either 'left', 'right', or 'both'
Remember:
Call get_goal_position to get the goal position of the gripper.
Stop position must always be greater than the goal position (the distance between gripper).
If a finger reaches a stop position, the goal position should be updated to the stop position.
If the grasp is incomplete, the gripper should open after re-adjusting the goal position.

```
def move_finger(position, velocity, finger='both')
```
position: the amount to move the finger inward or outward (in mm)
velocity: the velocity at which to move the finger in or at (in mm/s)
finger: which finger to move: 'left', 'right', or 'both'

Example answer code:
```
import numpy as np  # import numpy because we are using it below

# [REASONING] 
reset_parameters() # This is a new task so reset parameters to default; otherwise we don't need it
goal_position = get_goal_position()

[REASONING]
set_compliance(10, 3, 'both')
set_torque(1.0, 'both')
close_until_load(200, 1.5, 'both')
position = get_position()
set_goal_position(position - 5)
goal_position = get_goal_position()
close_gripper()
```

Remember:
1. Always format the code in code blocks. In your response all four functions above: set_torso_targets, set_foot_pos_parameters, execute_plan, should be called at least once.
2. Do not invent new functions or classes. The only allowed functions you can call are the ones listed above. Do not leave unimplemented code blocks in your response.
4. The only allowed library is numpy. Do not import or use any other library. If you use np, be sure to import numpy.
5. If you are not sure what value to use, just use your best judge. Do not use None for anything.
6. Do not calculate the position or direction of any object (except for the ones provided above). Just use a number directly based on your best guess.
7. If you see phrases like [REASONING], replace the entire phrase with a code comment explaining the grasp strategy and its relation to the following gripper commands.
"""


class PromptThinkerCoder(llm_prompt.LLMPrompt):
  """Prompt with both Motion Descriptor and Reward Coder."""

  def __init__(
      self,
      client: magpie_task_client.BarkourClient,
      executor: safe_executor.SafeExecutor,
  ):
    self._agent = client.agent()
    self._safe_executor = magpie_execution.BarkourSafeExecutor(executor)

    self.name = "Language2StructuredLang2GraspParameters"

    self.num_llms = 2
    self.prompts = [prompt_thinker, prompt_coder]

    # The coder doesn't need to keep the history as it only serves a purpose for
    # translating to code
    self.keep_message_history = [True, False]
    self.response_processors = [
        self.process_thinker_response,
        self.process_coder_response,
    ]
    self.code_executor = self.execute_code

  # process the response from thinker, the output will be used as input to coder
  def process_thinker_response(self, response: str) -> str:
    try:
      motion_description = (
          re.split(
              "end of description",
              re.split("start of description", response, flags=re.IGNORECASE)[
                  1
              ],
              flags=re.IGNORECASE,
          )[0]
          .strip("[")
          .strip("]")
          .strip()
          .strip("```")
      )
      return motion_description
    except Exception as _:  # pylint: disable=broad-exception-caught
      return response

  def process_coder_response(self, response):
    """Process the response from coder, the output will be the python code."""
    return process_code.process_code_block(response)

  def execute_code(self, code: str) -> None:
    print("ABOUT TO EXECUTE\n", code)
    mjpc_parameters = self._safe_executor.execute(code)
    self._agent.set_task_parameters(mjpc_parameters.task_parameters)
    self._agent.set_cost_weights(mjpc_parameters.cost_weights)
