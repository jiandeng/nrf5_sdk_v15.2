# CMSIS-DSP FFT算法分析
## 基本信息
* 量程: ±4g
* 采样率: 25Hz
* 分辨率: 1000/1024mg

## 时间/频域分辨率
* 频率分辨率 = 采样率 / N = 25 / N (Hz)
* 时间分辨率 = N / 采样率 (s)

| N   | 分析周期(s) | 频率分辨率(Hz) |
|-----|:-----------:|:--------------:|
| 64  | 2.5         | 0.4            |
| 128 | 5           | 0.2            |
| 256 | 10          | 0.1            |
| 512 | 20          | 0.05           |
| 1024| 40          | 0.25           |

## 空间开销
* input buffer: 4 \* 2 \* N = 8N (B)
* output  buffer: 4 \* N = 4N (B)
* total: 12N (B)

| N     | 空间开销(kB)   |
| ----- |:--------------:|
| 64    | 0.77           |
| 128   | 1.54           |
| 256   | 3.08           |
| 512   | 6.15           |
| 1024  | 12.3           |
| 2048  | FAIL           |

## 时间开销
* nRF52832上单次执行FFT

| N     | 时间开销(mS)   |
| ----- |:--------------:|
| 64    | 0.11           |
| 128   | 0.25           |
| 256   | 0.53           |
| 512   | 1.0            |
| 1024  | 2.3            |
| 2048  | FAIL           |

## FFT算法关键指标

| N   | 分析周期(s) | 频率分辨率(Hz) | 空间开销(kB)   | 时间开销(mS)   |
|-----|:-----------:|:--------------:|:--------------:|:--------------:|
| 64  | 2.5         | 0.4            | 0.77           | 0.11           |
| 128 | 5           | 0.2            | 1.54           | 0.25           |
| 256 | 10          | 0.1            | 3.08           | 0.53           |
| 512 | 20          | 0.05           | 6.15           | 1.0            |
| 1024| 40          | 0.25           | 12.3           | 2.3            |
| 2048| 80          | 0.125          | FAIL           | FAIL           |

## 滤波器时间开销
| N   | 3阶高通(us) | 3阶带通(us)    |
|-----|:-----------:|:--------------:|
| 64  | 36          | 52             |
| 128 | 64          | 104            |
| 256 | 136         | 204            |
| 512 | 264         | 400            |
| 1024| 528         | 800            |
| 2048| FAIL        | FAIL           |

* 3阶高通

```
    FILTER_FREQUENCY = 1
    SAMPLE_RATE = 25
    f = signal.butter(3, FILTER_FREQUENCY*2.0/SAMPLE_RATE, 'highpass', False, 'sos')
    print(f) # 打印滤波器系数
    w,h = signal.sosfreqz(f) # 绘制频率相应
    pd.DataFrame({'w': w/np.pi*SAMPLE_RATE/2, 'h': 10*np.log(abs(h))}).set_index('w').iplot(kind='scatter',theme='pearl',rangeslider=True)
    d = np.arange(0, 64) * 3.14159 / 1.29 # 滤波
    signal.sosfilt(f, d).round(6)
```
* 3阶带通

```
    FILTER_FREQUENCY = np.array([1.0, 7.5])
    SAMPLE_RATE = 25
    f = signal.butter(3, FILTER_FREQUENCY*2.0/SAMPLE_RATE, 'bandpass', False, 'sos')
    print(f) # 打印滤波系数
    w,h = signal.sosfreqz(f) # 绘制频率响应
    pd.DataFrame({'w': w/np.pi*SAMPLE_RATE/2, 'h': 10*np.log(abs(h))}).set_index('w').iplot(kind='scatter',theme='pearl',rangeslider=True)
    d = np.arange(0, 64) * 3.14159 / 1.29 # 滤波
    signal.lfilter(b, a, d).round(6)
```
