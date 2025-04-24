import torch
import torch.nn as nn
from collections import OrderedDict
import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import csv


arduino = serial.Serial(port='COM10',  baudrate=115200, timeout=.1)


checkpoint = torch.load("C:\\Users\\sonip\\Downloads\\checkpoint_9960000.pt", map_location="cpu")  # Adjust path/device
# print("Checkpoint keys:", checkpoint["loss_holonomic_with_rot"].keys()) 
class PolicyNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(6, 256),  # Input: 6D observation
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, 6)   # Output: 6 dimensions (3 for mean, 3 for scale)
        )
        
    def forward(self, x):
        return self.mlp(x)

# Initialize policy network
policy_net = PolicyNetwork()

# Extract and transform actor parameters
actor_params = OrderedDict()
for key in checkpoint["loss_holonomic_with_rot"]:
    if "actor_network_params" in key and ".mlp.params." in key:
        # Transform key: actor_network_params.module.0.module.0.mlp.params.0.weight -> mlp.0.weight
        parts = key.split(".mlp.params.")
        layer_part = parts[-1]  # Should be "0.weight", "0.bias", etc
        new_key = f"mlp.{layer_part}"
        actor_params[new_key] = checkpoint["loss_holonomic_with_rot"][key]

# Load transformed parameters
policy_net.load_state_dict(actor_params, strict=True)
policy_net.eval()



# Modified action function
def get_action(observation: list) -> np.ndarray:
    """Get deterministic action from policy network"""
    obs_tensor = torch.tensor(observation, dtype=torch.float32).unsqueeze(0)
    
    with torch.no_grad():
        output = policy_net(obs_tensor)
    
    # Split into mean and scale components
    # print('output',output)
    mean, _ = torch.chunk(output, 2, dim=-1)  # Take first 3 dimensions as mean
    # print('mean',mean)
    
    # Apply tanh to get actions in [-1, 1] range
    action = torch.tanh(mean)
    # print('action',action)
    action_scaled = np.multiply(action.squeeze(0).numpy(),[0,0.05,0.0001])
    
    return action_scaled

observation= [0,0,2.5,0,0,0]

action_log = []  # <-- list to store actions

#run simply this to get the forces state vector [x,y,Vx,Vy,theta,theta_dot] every 1 second do this
desiredvelocity_actualVelocity_distance_log = []  # To store [desiredVy, distanceY] pairs
observation_log = []
try:
    while True:
        actions = get_action(observation)
        print(actions)

        action_log.append(actions.tolist())  # Save action to list

        arduino.write(bytes(str(0.1), 'utf-8'))
        line = arduino.readline().decode().strip()

        try:
            values = list(map(float, line.split(',')))

            if len(values) == 9:
                observation = values[:6]
                desiredVy = values[6]
                distanceY = values[7]
                actualVy = values[8]

                observation_log.append(observation)
                desiredvelocity_actualVelocity_distance_log.append([desiredVy, distanceY,actualVy])

                print(f"Updated observation: {observation}")
                print(f"desiredVy: {desiredVy}, distanceY: {distanceY},ActualyVy: {actualVy}")
            else:
                print(f"Unexpected number of values ({len(values)}):", line)

        except ValueError:
            print("Invalid input:", line)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopped by user.")
    
    print("\nAction log:")
    for idx, a in enumerate(action_log):
        print(f"{idx}: {a}")
    
    print("\nVelocity & Distance log:")
    for idx, vd in enumerate(desiredvelocity_actualVelocity_distance_log):
        print(f"{idx}: desiredVy={vd[0]:.3f}, distanceY={vd[1]:.3f}, actualyVy={vd[2]:.3f}")

    # Save action log to CSV
    with open("action_log.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["action_0", "action_1", "action_2"])
        writer.writerows(action_log)
    print("Actions saved to action_log.csv")

    # Save velocity & distance log to CSV
    with open("velocity_distance_log.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["desiredVy", "distanceY"])
        writer.writerows(desiredvelocity_actualVelocity_distance_log)
    print("Velocity & distance saved to velocity_distance_log.csv")

