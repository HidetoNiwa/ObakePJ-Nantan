# ObakePJ

京都府南丹市にて開催されるお化け屋敷プロジェクトのお化けロボットのリポジトリ

* マイコン：NUCLEO-F303K8

## Pin Assign

| ピン名 | 機能名   | 備考      |
| ------ | -------- | --------- |
| PA0    | ADC1_1   |           |
| PA1    | ADC1_2   |           |
| PA2    | UART2_TX | USB UART  |
| PA3    | ADC1_4   |           |
| PA13   | SWDIO    |           |
| PA14   | SWCLK    |           |
| PA15   | UART2_RX | USB UART  |
| PB4    | TIM3_ch1 | PWM_A     |
| PB5    | TIM3_ch2 | PWM_B     |
| PB3    | GPIO     | LED Debug |

![image](https://os.mbed.com/media/uploads/bcostm/nucleo_f303k8_2017_10_10.png)

### 参考

[NUCLEO-F303K8(mbed)](https://os.mbed.com/platforms/ST-Nucleo-F303K8/)

## UART

実行状況Debug用

115200bps

### 参考

[STM32でUARTをやってみる3](https://gsmcustomeffects.hatenablog.com/entry/2017/03/23/125604)

## ADC

パラメータ調整用にボリュームを搭載．この値を読み取るために使用

### 参考

[Nucleo開発メモ（ADCを使う）](https://b.meso.tokyo/post/173610335934/stm32-nucleo-adc)

## PWM

* Prescaler:0
* Counter Period:999
* APB2 timer clocks : 64MHz

PWM周波数->64MHz/((0+1)*999+1)=64kHz

###　参考
[STM32CubeIDEを使ってみよう How To STM32CubeIDE 日本語版 (13) PWMでサーボモーター（SERVO)を動かしてみよう(Qiita)](https://qiita.com/usashirou/items/2d0fedf59a3cef083b87)

[0626 SW4STM32＆CubeMXを用いてNucleoでPWMしてみる](http://ropot.hatenablog.com/entry/2017/06/26/212615)
