from matplotlib import pyplot as plt
from scipy.stats import norm
import csv

f = open("gail-res.txt", "r")

def log_obs(actual, predicted, stdev):
  return norm(actual, stdev).logpdf(predicted)

def motor_model(prev_la, dec_max, acc_max):    
  acc0 = min(prev_la+1, acc_max)
  acc2 = max(prev_la-1, dec_max)
  acc1 = acc0 if prev_la<=0 else acc2
  return [acc0, acc1, acc2]
    

def motor_model_closest(pred, prev, dec_max, acc_max):
  possible = motor_model(prev, dec_max, acc_max)
  possible_mag = [abs(p - pred) for p in possible]
  if pred <= possible[0] and pred >= possible[2]:
    return pred
  else:
    return possible[possible_mag.index(min(possible_mag))]

actual_acc = []
pred_acc = []
pred_acc_raw = []
n = -1
info = []
prev = 0
res_sum_train = 0
res_sum_test = 0
res_sum_all = 0

while(True):
  line = f.readline()
  if "average" in line:
    break
  if "iter" in line:
    n = int(line[4:])
    print("experiment "+str(n))
    f2 = open("data-1d/data"+str(n)+".csv")
    reader = csv.reader(f2)
    next(reader)
    info = next(reader)
    print([info[3], info[4], info[5], info[7]])
  elif " " in line:
    vals = line.split(" ")
    raw = float(vals[0])
    if len(actual_acc) > 0:
      prev = actual_acc[-1]
    pred_acc.append(motor_model_closest(raw, prev, float(info[3]), float(info[4])))
    pred_acc_raw.append(raw)
    actual_acc.append(float(vals[1]))
  elif line == "0\n":
    log_obs_vals = [log_obs(actual_acc[i], pred_acc[i], 0.5) for i in range(len(actual_acc))]
    # print(log_obs_vals)
    res = sum(log_obs_vals)/len(actual_acc)
    res_sum_all += res
    if n < 10:
      res_sum_train += res
    else:
      res_sum_test += res
    print(res)
    plt.plot(pred_acc)
    plt.plot(actual_acc)
    plt.savefig("plots/baseline_1_exp_proc_"+str(n))
    plt.close()
    plt.plot(pred_acc_raw)
    plt.plot(actual_acc)
    plt.savefig("plots/baseline_1_exp_raw_"+str(n))
    plt.close()
    actual_acc = []
    pred_acc = []
    pred_acc_raw = []
    prev = 0
print("AVG LOG OBS: train test all")
print(res_sum_train / 10)
print(res_sum_test / 20)
print(res_sum_all / 30)






