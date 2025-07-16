
import os
import pandas as pd
import numpy as np
import torch
from torch import nn
from torch.utils.data import TensorDataset, DataLoader
import matplotlib.pyplot as plt
import pickle
from sklearn.metrics import mean_squared_error

# ---- Configuration ----
CSV_FILE = "gem_data_log.csv"
WHEELBASE = 1.75
EPOCHS = 100
BATCH_SIZE = 32
LEARNING_RATE = 0.001
OUTPUT_DIR = "residual_model_output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ---- Load CSV ----
df = pd.read_csv(CSV_FILE)
df['dt'] = df['timestamp'].diff().fillna(0.04)

# ---- Extract Variables ----
x = df['odom_x'].values[:-1]
y = df['odom_y'].values[:-1]
yaw = df['odom_yaw'].values[:-1]
v = df['speed'].values[:-1]
delta = df['steering_angle'].values[:-1]
dt = df['dt'].values[:-1]

x_next_true = df['odom_x'].values[1:]
y_next_true = df['odom_y'].values[1:]
yaw_next_true = df['odom_yaw'].values[1:]

# ---- Bicycle Model Prediction ----
x_pred = x + v * np.cos(yaw) * dt
y_pred = y + v * np.sin(yaw) * dt
yaw_pred = yaw + (v / WHEELBASE) * np.tan(delta) * dt

# ---- Yaw Angle Correction ----
def angle_diff(a, b):
    return (a - b + np.pi) % (2 * np.pi) - np.pi

# ---- Residuals ----
res_x = x_next_true - x_pred
res_y = y_next_true - y_pred
res_yaw = angle_diff(yaw_next_true, yaw_pred)

# ---- Normalize Inputs ----
inputs = np.stack([x, y, yaw, v, delta], axis=1)
mean = inputs.mean(axis=0)
std = inputs.std(axis=0) + 1e-6
inputs_norm = (inputs - mean) / std
residuals = np.stack([res_x, res_y, res_yaw], axis=1)

# Save normalization params
with open(os.path.join(OUTPUT_DIR, "residual_input_norm.pkl"), "wb") as f:
    pickle.dump({'mean': mean, 'std': std}, f)

# ---- Dataset Preparation ----
indices = np.random.permutation(len(inputs_norm))
n = int(0.8 * len(inputs_norm))

X_train = inputs_norm[indices[:n]]
Y_train = residuals[indices[:n]]
X_test = inputs_norm[indices[n:]]
Y_test = residuals[indices[n:]]

X_train_tensor = torch.tensor(X_train, dtype=torch.float32)
Y_train_tensor = torch.tensor(Y_train, dtype=torch.float32)
X_test_tensor = torch.tensor(X_test, dtype=torch.float32)
Y_test_tensor = torch.tensor(Y_test, dtype=torch.float32)

train_ds = TensorDataset(X_train_tensor, Y_train_tensor)
train_dl = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)

# ---- Residual Neural Network ----
class ResidualNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(5, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 3)
        )

    def forward(self, x):
        return self.net(x)

model = ResidualNN()
optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
loss_fn = nn.MSELoss()

# ---- Training Loop with RMSE Logging ----
rmse_train_list = []
rmse_val_list = []

for epoch in range(EPOCHS):
    model.train()
    for xb, yb in train_dl:
        pred = model(xb)
        loss = loss_fn(pred, yb)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    # RMSE calculation
    model.eval()
    with torch.no_grad():
        train_pred = model(X_train_tensor)
        val_pred = model(X_test_tensor)
        train_rmse = torch.sqrt(torch.mean((train_pred - Y_train_tensor) ** 2)).item()
        val_rmse = torch.sqrt(torch.mean((val_pred - Y_test_tensor) ** 2)).item()
        rmse_train_list.append(train_rmse)
        rmse_val_list.append(val_rmse)

    if epoch % 10 == 0:
        print(f"Epoch {epoch} | Train RMSE: {train_rmse:.6f} | Val RMSE: {val_rmse:.6f}")

# ---- Save Model ----
torch.save(model.state_dict(), os.path.join(OUTPUT_DIR, "residual_model.pth"))

# ---- Evaluation ----
with torch.no_grad():
    pred_test = model(X_test_tensor)
    rmse = torch.sqrt(torch.mean((pred_test - Y_test_tensor) ** 2, dim=0))
    print(f"\nTest RMSE (dx, dy, dyaw): {rmse.tolist()}")
    print("\nSample predictions vs ground truth:")
    print("Predicted:", pred_test[:3].numpy())
    print("Ground Truth:", Y_test[:3])

# ---- TorchScript Export ----
example_input = torch.randn(1, 5)
traced_model = torch.jit.trace(model, example_input)
traced_model.save(os.path.join(OUTPUT_DIR, "residual_model_traced.pt"))
print("Saved TorchScript model to residual_model_traced.pt")

# ---- Plot RMSE Curves ----
plt.figure()
plt.plot(rmse_train_list, label="Train RMSE")
plt.plot(rmse_val_list, label="Val RMSE")
plt.xlabel("Epoch")
plt.ylabel("RMSE")
plt.title("Residual Model RMSE Over Epochs")
plt.legend()
plt.grid()
plt.savefig(os.path.join(OUTPUT_DIR, "residual_rmse_plot.png"))
plt.close()

# ---- Plot Input vs Output Comparison ----
pred_test_np = pred_test.numpy()
Y_test_np = Y_test_tensor.numpy()
time = np.arange(len(pred_test_np))

fig, axs = plt.subplots(3, 1, figsize=(10, 8))
labels = ['dx', 'dy', 'dyaw']

for i in range(3):
    axs[i].plot(time, Y_test_np[:, i], label='True')
    axs[i].plot(time, pred_test_np[:, i], '--', label='Predicted')
    axs[i].set_ylabel(labels[i])
    axs[i].legend()
    axs[i].grid()

axs[2].set_xlabel('Sample Index')
plt.suptitle("Residual Model: True vs Predicted Residuals")
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "input_output_comparison.png"))
plt.show()

