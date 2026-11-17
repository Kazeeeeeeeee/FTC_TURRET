# BioBuzz 自動對準炮臺、移動射擊（Shoot-on-the-Move, SOTM）與 Limelight 3A 視覺對接系統指南

本系統為 FTC "BioBuzz" 賽季設計，提供高精度、模組化的**自動對準旋轉炮臺與可變速滾筒發射器系統**。系統全流程採用英吋（Inches）與度/弧度（Degrees/Radians）進行幾何與動態計算，整合 goBILDA Pinpoint 雙輪定位儀，具備剛體運動學（Rigid-Body Kinematics）、延遲前饋補償（Latency Lookahead）、3 步疊代虛擬目標求解器（Iterative Virtual Target Solver）、籃筐進球角度接收半球檢查（Basket Entry Angle Check）、FTC Dashboard 即時可視化場地畫布（Field Canvas Overlay）與 Limelight 3A 視覺動態對接介面。

---

## 📁 系統架構與檔案說明

| 檔案名稱 | 檔案路徑 | 說明與角色 |
|---|---|---|
| [`TurretCalibrationOpMode.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretCalibrationOpMode.java) | `TeamCode/...` | **Step 3 校正工具**：無阻力手動旋轉炮臺，自動測量並計算 `TURRET_DEGREES_PER_TICK` 與 `TURRET_ENCODER_REVERSED` 極性。 |
| [`AutoTurretShooter.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/AutoTurretShooter.java) | `TeamCode/...` | **核心子系統 Class**：預填全場幾何參數、Dashboard `@Config` 可調參數、Pinpoint 速度低通濾波、剛體偏移運動學、SOTM 3 步疊代求解器、彈道物理與 LUT 計算、軟限位保護、FTC Dashboard Field Canvas 畫布輸出及 Limelight 3A 視覺對接 Hook。 |
| [`BioBuzzTeleOp.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzTeleOp.java) | `TeamCode/...` | **競賽級 TeleOp 示範**：整合全向/麥克納姆輪驅動，支援一鍵切換 Sample 大小、目標籃筐、SOTM 補償，雙路 Telemetry 同步發送至遙控器與 Dashboard，並在 `isReadyToShoot()` 成立時自動觸發進彈與震動回饋。 |
| [`PoseStorage.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PoseStorage.java) | `TeamCode/...` | **座標傳遞 Class**：將 Autonomous 結束時的機器人座標無縫傳遞給 TeleOp。 |

---

## ⚙️ 步驟零：手動順時針 360° 炮臺 Encoder 校正教學

在計算炮臺旋轉角度時，無需手動計算齒輪比與馬達 Encoder 每圈 Ticks。本系統提供直觀的**手動順時針旋轉校正 OpMode** (`TurretCalibrationOpMode.java`)：

### 校正步驟：
1. **執行 OpMode**：在 Driver Station 上選擇 **`1. Turret 360 Manual Calibration`** 並按 INIT。
   - 系統會將 `turretMotor` 設定為**無電磁阻力模式**（`RUN_WITHOUT_ENCODER` + `ZeroPowerBehavior.FLOAT` + `setPower(0.0)`），此時馬達完全無電磁煞車，可以輕鬆用手旋轉炮臺。
2. **選擇目標角度 (預設 360°)**：
   - 使用 Gamepad 1 **D-Pad Up/Down** 可在 `360.0°`、`180.0°` 與 `90.0°` 之間切換。若炮臺已接線無法轉滿 360 度，請切換至 `180.0°` 或 `90.0°` 模式。
3. **對齊基準線並歸零**：
   - 用手將炮臺轉至正前方物理基準標記。
   - 按下 Gamepad 1 **`[A]`（Cross）** 鍵歸零 Encoder。
4. **手動順時針旋轉**：
   - 用手將炮臺**順時針（Clockwise / CW，從上往下看往右轉）** 轉滿 360 度（或 180° / 90°）。
5. **鎖定並讀取結果**：
   - 按下 Gamepad 1 **`[B]`（Circle）** 鍵鎖定最終讀數。
   - Telemetry 畫面將自動顯示計算結果與極性判斷：
     ```text
     COPY & PASTE INTO AutoTurretShooter.java:
     public static double TURRET_DEGREES_PER_TICK = 0.12500000;
     public static boolean TURRET_ENCODER_REVERSED = false;
     ```
6. **複製貼回代碼**：
   - 將畫面輸出的 `TURRET_DEGREES_PER_TICK` 與 `TURRET_ENCODER_REVERSED` 複製貼回 `AutoTurretShooter.java` 頂部對應欄位即可！

> 💡 **極性自動判定原理**：根據本系統座標定義，頂視順時針旋轉（往右）為**負角度（$-360^\circ$）**。若順時針旋轉時 Encoder 讀數為負數，則 `TURRET_ENCODER_REVERSED = false`；若讀數為正數，則 `TURRET_ENCODER_REVERSED = true`（代表 Encoder 讀數反向）。

---

## Step 1：Control Hub 硬體接線與 Robot Configuration 設定

在 **FTC Driver Station** 遙控器 APP 的 `Configure Robot` 檔案中，請務必建立並命名以下硬體裝置：

| Driver Station 點選名稱 | 裝置種類與類別 (Type) | 推薦 REV Hub 實體端口 | 代碼對應 (`hardwareMap.get`) | 接線與說明 |
|---|---|---|---|---|
| `pinpoint` | **I2C Device** $\rightarrow$ `goBILDA Pinpoint Odometry Computer` | **I2C Bus Port 1, 2 或 3** | `hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint")` | ⚠️ **切勿接在 Bus 0**（Control Hub Bus 0 內建 IMU 佔用）。 |
| `turretMotor` | **Motors** $\rightarrow$ `REV Expansion Hub Motor` (`DcMotorEx`) | Motor Port 0 | `hardwareMap.get(DcMotorEx.class, "turretMotor")` | 必須接 Encoder 線，用於炮臺角度控制。 |
| `shooterMotor` | **Motors** $\rightarrow$ `REV Expansion Hub Motor` (`DcMotorEx`) | Motor Port 3 | `hardwareMap.get(DcMotorEx.class, "shooterMotor")` | 必須接 Encoder 線，支援閉迴路轉速控制 (`RUN_USING_ENCODER`)。 |
| `feedServo` *(選配)* | **Servos** $\rightarrow$ `Servo` (Standard Servo) | Servo Port 0 或 1 | `hardwareMap.get(Servo.class, "feedServo")` | 推球/進彈伺服馬達。 |
| `limelight` *(預留)* | **Limelight3A** (Vision Sensor) | Control Hub USB 端口 | `hardwareMap.get(Limelight3A.class, "limelight")` | 預留視覺鏡頭介面。 |

---

## Step 2：核對已內建數據 & 測量最後 4 個機械參數

### 2.1 已內建於代碼中的數據（已預填於 `AutoTurretShooter.java`）

以下數據已根據 BioBuzz 官方規格與幾何計算預填於代碼中：

| 參數項目 | 內建設定值 | 單位 | 幾何說明 |
|---|---|---|---|
| 全場尺寸 | $144.0 \times 144.0$ | 英吋 (in) | 左下角 $(0.0, 0.0)$、右上角 $(144.0, 144.0)$ |
| 機器人初始中心座標 | `(138.0, 30.2)` | 英吋 (in) | 機器人底盤幾何中心開局位置 |
| 機器人初始朝向 | `180.0` | 度 (deg) | 面朝左側（$-X$ 方向，即 $\pi$ 弧度） |
| 預設目標籃筐 (`RED_RIGHT`) | `(84.0, 60.0, 高 56.0)` | 英吋 (in) | 籃筐開口朝右 ($0.0^\circ$) 接收 Sample |
| **Odometry 實測解析度** | `40.0 / 23919.0` | 英吋/Tick | **實測 40 英吋 = 23,919 Ticks** ($\approx 0.0016723\text{ in/tick} \approx 597.975\text{ ticks/in}$) |
| Pinpoint Odometry Pod Offsets | `setOffsets(1.5, -3.5)` | 英吋 (in) | 根據底盤幾何中心 `(6.5, 6.0)` 換算：前後 pod 偏置 $+1.5"$，左右 pod 偏置 $-3.5"$ |

---

### 2.2 上機前你必須親自拿尺測量的 4 個參數

打開 `AutoTurretShooter.java` 頂部，親自拿尺測量並更新以下 4 組硬體參數：

1. **炮臺旋轉軸心偏移量 `(TURRET_OFFSET_FORWARD_IN, TURRET_OFFSET_LEFT_IN)`**：
   - 從機器人底盤中心 `(6.5, 6.0)` 量至炮臺旋轉圓心的距離。
   - **正負號規則**：向前為正 $+$、向後為負 $-$；向左為正 $+$、向右為負 $-$（單位：英吋）。
2. **發射口幾何高度 `SHOOTER_RELEASE_HEIGHT_IN`**：
   - Sample 離開滾筒瞬間，球心離地面的垂直高度（單位：英吋）。
3. **滾筒半徑 `ROLLER_RADIUS_IN` 與發射仰角 `LAUNCH_ANGLE_DEG`**：
   - 射擊滾筒半徑（英吋，預設 `2.0` in）與導軌固定仰角（度，預設 `50.0` 度）。
4. **炮臺防纏線軟限位 `MIN_TURRET_ANGLE_DEG` 與 `MAX_TURRET_ANGLE_DEG`**：
   - 考量電線長度後，炮臺可往右轉（負值，如 `-135.0°`）與往左轉（正值，如 `+135.0°`）的最大安全角度。

---

## Step 3：執行「手動順時針 360°」炮臺 Encoder 校正

請參閱頂部【步驟零】操作 `TurretCalibrationOpMode.java`，取得 `TURRET_DEGREES_PER_TICK` 與 `TURRET_ENCODER_REVERSED` 後填回代碼。

---

## Step 4：每次比賽/測試前的「擺車與歸零 (Homing)」標準動作

在每次點擊 Driver Station 的 `INIT` 按鈕之前，隊員**必須**完成以下 2 個物理動作：

1. **車體對位 (Robot Placement)：**
   - 將機器人底盤幾何中心對準場地 `(138.0, 30.2)` 英吋處，車頭正對左側（$-X$ 方向，即 $180^\circ$）。
2. **炮臺歸零 (Turret Homing)：**
   - 用手將炮臺旋轉至正對車頭前方（對齊 Step 3 的 $0^\circ$ 物理膠帶線）。
3. **點擊 INIT：**
   - 按下 `INIT`，系統會呼叫 `STOP_AND_RESET_ENCODER` 將當前位置鎖定為 $0$ Ticks（即車頭正前方）。

---

## Step 5：第一階段實測——靜態自動瞄準與 Dashboard 秒級轉速調校

### 5.1 連接 FTC Dashboard (`http://192.168.43.1:8080/dash`)
1. 將電腦 Wi-Fi 連至 Robot Controller Wi-Fi 熱點。
2. 在瀏覽器打開 `http://192.168.43.1:8080/dash`。

### 5.2 靜態瞄準驗證與免刷代碼秒級調參
1. 在 `AutoTurretShooter.java` 中確認 `ENABLE_SHOOT_ON_THE_MOVE = false`（或在 TeleOp 中按 Gamepad1 `[X]` 關閉 SOTM）。
2. 按下 `START` 後機器人保持靜止。檢查 Telemetry 的數據：
   - 目標籃筐 `RED_RIGHT`：`(84.0, 60.0)`
   - 靜態 2D 距離：約 **`61.68 inch`** ($\sqrt{(-54)^2 + (29.8)^2}$)
   - 炮臺相對角度：約 **`-28.89°`**（往車頭右側偏轉約 29 度正對籃筐）。
3. **在 Dashboard 右側 `Configuration` 面板中展開 `AutoTurretShooter`**：
   - **免重新編譯上傳代碼**：直接在瀏覽器修改 `SMALL_SAMPLE_SLIP_FACTOR`（預設 `0.65`）或 `LARGE_SAMPLE_SLIP_FACTOR`（預設 `0.75`），按下 **Save** 立即生效！
   - 若拋物線偏短：調大對應 Sample 的 Slip Factor。
   - 若拋物線飛過頭：調小對應的 Slip Factor。
4. **即時折線圖 (Graphing)**：在 Dashboard 左下角勾選 `targetShooterRPM` 與 `actualShooterRPM`，即時觀察發射馬達閉迴路轉速追隨曲線。

---

## Step 6：第二階段實測——開啟「移動中射擊 (SOTM)」與 Dashboard 場地視覺化除錯

開啟 `ENABLE_SHOOT_ON_THE_MOVE = true`，進行移動中發射調校：

```mermaid
flowchart TD
    A["步驟 6.1：觀察 Dashboard Field View 畫布 (Field Canvas)"] -->|藍圈=機器人, 紫線=瞄準光束, 紅圈=真實籃筐 (84,60), 綠圈=虛擬籃筐| B["步驟 6.2：調校橫移提前量 (LATENCY_COMPENSATION_SEC)"]
    B -->|左右平移發射，修正綠色虛擬籃筐落後/超前| C["步驟 6.3：調校前後距離阻力補償 (TOF_DRAG_SCALAR)"]
    C -->|進退發射，微調空氣阻力補償| D["完成 SOTM 移動射擊調校"]
```

### 6.1 利用 Dashboard Field View 2D 場地畫布即時除錯
在 Dashboard 左上角的 **Field View（場地視圖）** 中，本系統會即時繪製：
1. **藍色圓圈與車頭線**：代表機器人即時座標 `(robotX, robotY)` 與朝向。
2. **紫色瞄準光束**：從炮臺中心延伸至虛擬目標。
3. **紅色/藍色圓圈**：代表場地真實籃筐 `RED_RIGHT (84.0, 60.0)`。
4. **綠色圓圈與十字準星 (Virtual Target)**：代表 SOTM 求解器計算出的**虛擬目標 `(virtualX, virtualY)`**。當你推動機器人時，你會直觀看到綠色準星根據車速動態向前/向後平移！

### 6.2 秒級調校移動射擊三大關鍵參數
在 Dashboard 直接修改數字並按 `Save`：
1. **`VELOCITY_FILTER_ALPHA`（預設 `0.20`）**：低通濾波係數。若移動時炮臺發抖，調低該值（例如 `0.10`）。
2. **`LATENCY_COMPENSATION_SEC`（預設 `0.08` 秒）**：延遲前饋補償。若平移發射球總是偏向車子剛經過的位置（落後），調大該值（例如 `0.10` 秒）。
3. **`SMALL/LARGE_SAMPLE_TOF_SCALAR`（預設 小 `1.05` / 大 `1.10`）**：飛行時間阻力修正。若高速後退發射球偏短，微幅調高（例如 `1.15`）。

---

## Step 7：如何在你自己 TeleOp 中使用這套系統（完整 Copy-Paste 教學）

隊員只需 4 個步驟即可把系統整合進現有 TeleOp：

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "My Competition TeleOp", group = "Competition")
public class MyCompetitionTeleOp extends LinearOpMode {

    // 1. 宣告 AutoTurretShooter 物件
    private final AutoTurretShooter turretShooter = new AutoTurretShooter();

    @Override
    public void runOpMode() {
        // 2. 初始化子系統 (自動加載 (138.0, 30.2, 朝左 180°) 初始姿態與 (1.5, -3.5) Offsets)
        turretShooter.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            boolean spinUpShooter = gamepad1.right_trigger > 0.1;

            // 3. Loop 主迴圈一行更新：自動提取姿態、SOTM 虛擬目標求解、炮臺旋轉與滾筒轉速控制
            AutoTurretShooter.AimResult aim = turretShooter.updateAutoAim(
                    AutoTurretShooter.SampleType.SMALL,
                    spinUpShooter,
                    true // 啟用 SOTM 移動射擊
            );

            // 4. 自動進彈：僅在 isReadyToShoot() 成立時才觸發推球伺服馬達
            if (spinUpShooter && turretShooter.isReadyToShoot()) {
                turretShooter.actuateFeedServo(true);
            } else {
                turretShooter.actuateFeedServo(false);
            }

            // 發送詳細 telemetry 至遙控器與 Dashboard
            turretShooter.sendTelemetry(telemetry);
        }
    }
}
```

---

## Step 8：常見問題排錯表 (Troubleshooting Table) & Limelight 3A 未來擴充指南

### 8.1 常見問題排錯表

| 異常現象 | 可能原因 | 解決方法 |
|---|---|---|
| **炮臺往反方向狂轉** | `TURRET_ENCODER_REVERSED` 極性設定錯誤。 | 重新執行 Step 3 `TurretCalibrationOpMode` 校正極性。 |
| **炮臺轉到死區卡住** | 轉向超過 `MIN_TURRET_ANGLE_DEG` / `MAX_TURRET_ANGLE_DEG`。 | 檢查 `AimResult.isTurretClamped` 是否為 `true`，調整擺車位置。 |
| **滾筒轉速上不去 / 不轉** | `shooterMotor` 未接 Encoder 線或 PIDF 參數未設定。 | 檢查馬達 Encoder 線接法，確認模式為 `RUN_USING_ENCODER`。 |
| **`isReadyToShoot()` 一直為 `false`** | 1. 距離超出 `18.0" ~ 144.0"` 範圍<br>2. 機器人移動速度超標 ($>60$ in/s)<br>3. 籃筐進球角度偏向背側 ($>75^\circ$)。 | 檢查 Telemetry 的 `blockedReason` 欄位以獲取具體阻塞原因。 |

---

### 8.2 Limelight 3A 未來視覺對接指南

子系統已內建 Limelight 3A 視覺介面 Hook：
- 在 `AutoTurretShooter.java` 中，`updateValidBasketsFromLimelight(limelight)` 會自動讀取 Limelight 3A 的視覺識別數據。
- 視覺演算法可透過 `setBasketShootable(TargetBasketMode mode, boolean isShootable)` 動態開啟/關閉 4 個籃筐的可射擊狀態，主系統將自動挑選距離最近且 `isShootable = true` 的有效籃筐進行瞄準！
