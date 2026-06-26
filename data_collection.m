clear; clc; close all;

%% ========== 基本设置 ==========
PORT = "COM9";
BAUD = 115200;
VREF = 5.0;   % Arduino UNO 默认 5V；如果是 3.3V 板子，改成 3.3

timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));

RAW_FILE     = "four_channel_emg_raw_" + timestamp + ".csv";
VOLT_FILE    = "four_channel_emg_voltage_" + timestamp + ".csv";
SUMMARY_FILE = "four_channel_emg_summary_" + timestamp + ".csv";

FIG_VOLTAGE  = "four_channel_voltage_over_time_" + timestamp + ".png";
FIG_FEATURE  = "four_channel_feature_over_time_" + timestamp + ".png";
FIG_SCORE    = "press_release_score_" + timestamp + ".png";

% 采集 5 组完整驾驶踏板周期
N_TRIALS = 5;

% 动作时间：舒适、低疲劳
NEUTRAL_SEC = 5.0;     % 中立位
PRESS_SEC   = 3.0;     % 踩下
HOLD_SEC    = 3.0;     % 保持
RELEASE_SEC = 3.0;     % 释放
REST_SEC    = 6.0;     % 休息

CSV_HEADER = "time_ms,label,A0_raw,A1_raw,A2_raw,A3_raw,A0_feature,A1_feature,A2_feature,A3_feature,A0_baseline,A1_baseline,A2_baseline,A3_baseline";

%% ========== 连接 Arduino ==========
disp("正在连接 Arduino...");
disp("请确认 Arduino 串口监视器已经关闭。");

availablePorts = serialportlist("available");
disp("当前可用端口：");
disp(availablePorts);

if ~any(availablePorts == PORT)
    error("未检测到 COM9。请检查 Arduino 是否连接，或确认端口是否为 COM9。");
end

s = serialport(PORT, BAUD, "Timeout", 1);
configureTerminator(s, "LF");

disp("连接成功：COM9");
disp("请保持腿部完全放松，等待 Arduino 自动校准...");
pause(6);

% 清空校准期间残留数据
flush(s);

fid = fopen(RAW_FILE, "w");
if fid == -1
    error("无法创建数据文件，请检查 MATLAB 当前文件夹权限。");
end

% MATLAB 直接写表头，避免串口表头丢失
fprintf(fid, "%s\n", CSV_HEADER);

%% ========== 实验说明 ==========
disp("======================================");
disp("四通道驾驶踏板 EMG 采集实验");
disp("本次采集 5 组完整周期");
disp("动作周期：中立位 → 踩下 → 保持 → 释放 → 休息");
disp("");
disp("A0：释放主通道 / 小腿前侧");
disp("A1：释放辅助通道 / 小腿前侧辅助");
disp("A2：踩下主通道 / 小腿后侧");
disp("A3：踩下辅助通道 / 小腿后侧辅助");
disp("");
disp("动作轻到中等力度即可，不要用最大力。");
disp("如有不适，立即停止。");
disp("======================================");
input("准备好后按回车开始实验：", "s");

%% ========== 初始中立位 ==========
runPhase(s, fid, 'n', 5.0, ...
    "初始中立位", "脚自然放松，不主动用力");

%% ========== 主实验：5组 ==========
for k = 1:N_TRIALS
    fprintf("\n========== 第 %d / %d 组 ==========\n", k, N_TRIALS);

    runPhase(s, fid, 'n', NEUTRAL_SEC, ...
        "中立位", "脚自然放松，保持准备位");

    runPhase(s, fid, 'p', PRESS_SEC, ...
        "踩下", "轻轻向下踩踏板 / 趾屈");

    runPhase(s, fid, 'h', HOLD_SEC, ...
        "保持", "保持当前位置，不继续加力");

    runPhase(s, fid, 'u', RELEASE_SEC, ...
        "释放", "抬脚，释放踏板 / 背屈");

    runPhase(s, fid, 'r', REST_SEC, ...
        "休息", "完全放松，准备下一组");
end

fclose(fid);
clear s;

disp("======================================");
disp("采集完成。");
disp("原始数据已保存：");
disp(RAW_FILE);
disp("======================================");

%% ========== 读取数据 ==========
T = readtable(RAW_FILE);

if height(T) == 0
    error("CSV 文件为空。请检查 Arduino 是否正常输出数据。");
end

labels = string(T.label);
t = (T.time_ms - T.time_ms(1)) / 1000;

%% ========== 标签数量检查 ==========
disp("========== 标签数量检查 ==========");
fprintf("总数据行数：%d\n", height(T));
fprintf("NEUTRAL: %d\n", sum(labels == "NEUTRAL"));
fprintf("PRESS:   %d\n", sum(labels == "PRESS"));
fprintf("HOLD:    %d\n", sum(labels == "HOLD"));
fprintf("RELEASE: %d\n", sum(labels == "RELEASE"));
fprintf("REST:    %d\n", sum(labels == "REST"));

%% ========== ADC 转电压 ==========
T.A0_voltage = T.A0_raw / 1023 * VREF;
T.A1_voltage = T.A1_raw / 1023 * VREF;
T.A2_voltage = T.A2_raw / 1023 * VREF;
T.A3_voltage = T.A3_raw / 1023 * VREF;

%% ========== 计算 PressScore / ReleaseScore ==========
% 最简便高效版：
% ReleaseScore = A0 + A1 前侧组平均
% PressScore   = A2 + A3 后侧组平均

T.ReleaseScore_raw = (T.A0_feature + T.A1_feature) / 2;
T.PressScore_raw   = (T.A2_feature + T.A3_feature) / 2;

%% ========== 归一化 Score ==========
baseIdx    = labels == "NEUTRAL" | labels == "REST";
pressIdx   = labels == "PRESS";
releaseIdx = labels == "RELEASE";
holdIdx    = labels == "HOLD";

releaseBase95 = percentileLocal(T.ReleaseScore_raw(baseIdx), 95);
pressBase95   = percentileLocal(T.PressScore_raw(baseIdx), 95);

releasePeak90 = percentileLocal(T.ReleaseScore_raw(releaseIdx), 90);
pressPeak90   = percentileLocal(T.PressScore_raw(pressIdx), 90);

% 避免除零
releaseRange = max(releasePeak90 - releaseBase95, 1);
pressRange   = max(pressPeak90 - pressBase95, 1);

T.ReleaseScore = (T.ReleaseScore_raw - releaseBase95) / releaseRange;
T.PressScore   = (T.PressScore_raw - pressBase95) / pressRange;

% 限制在合理范围，便于图像展示
T.ReleaseScore(T.ReleaseScore < 0) = 0;
T.PressScore(T.PressScore < 0) = 0;

writetable(T, VOLT_FILE);
disp("带电压和 Score 的数据已保存：");
disp(VOLT_FILE);

%% ========== 图1：四通道电压随时间 ==========
figure("Name", "Four-channel Voltage", "Color", "w");

plot(t, T.A0_voltage, "LineWidth", 1.2); hold on;
plot(t, T.A1_voltage, "LineWidth", 1.2);
plot(t, T.A2_voltage, "LineWidth", 1.2);
plot(t, T.A3_voltage, "LineWidth", 1.2);

xlabel("Time (s)");
ylabel("Voltage (V)");
title("Four-channel EMG Voltage Over Time");
legend("A0 Release Main", "A1 Release Aux", "A2 Press Main", "A3 Press Aux", ...
    "Location", "best");
grid on;

markPhaseChanges(t, labels);
saveas(gcf, FIG_VOLTAGE);

disp("四通道电压图已保存：");
disp(FIG_VOLTAGE);

%% ========== 图2：四通道 Feature 随时间 ==========
figure("Name", "Four-channel Feature", "Color", "w");

plot(t, T.A0_feature, "LineWidth", 1.2); hold on;
plot(t, T.A1_feature, "LineWidth", 1.2);
plot(t, T.A2_feature, "LineWidth", 1.2);
plot(t, T.A3_feature, "LineWidth", 1.2);

xlabel("Time (s)");
ylabel("EMG Feature");
title("Four-channel EMG Feature Over Time");
legend("A0 Release Main", "A1 Release Aux", "A2 Press Main", "A3 Press Aux", ...
    "Location", "best");
grid on;

markPhaseChanges(t, labels);
saveas(gcf, FIG_FEATURE);

disp("四通道特征图已保存：");
disp(FIG_FEATURE);

%% ========== 图3：PressScore / ReleaseScore ==========
figure("Name", "PressScore vs ReleaseScore", "Color", "w");

plot(t, T.PressScore, "LineWidth", 1.6); hold on;
plot(t, T.ReleaseScore, "LineWidth", 1.6);

xlabel("Time (s)");
ylabel("Normalized Score");
title("PressScore and ReleaseScore Over Time");
legend("PressScore = mean(A2,A3)", "ReleaseScore = mean(A0,A1)", ...
    "Location", "best");
grid on;

markPhaseChanges(t, labels);
saveas(gcf, FIG_SCORE);

disp("PressScore / ReleaseScore 图已保存：");
disp(FIG_SCORE);

%% ========== 关键指标输出 ==========
pressScore_press90     = percentileLocal(T.PressScore(pressIdx), 90);
releaseScore_press90   = percentileLocal(T.ReleaseScore(pressIdx), 90);
releaseScore_release90 = percentileLocal(T.ReleaseScore(releaseIdx), 90);
pressScore_release90   = percentileLocal(T.PressScore(releaseIdx), 90);

pressScore_hold90      = percentileLocal(T.PressScore(holdIdx), 90);
releaseScore_hold90    = percentileLocal(T.ReleaseScore(holdIdx), 90);

fprintf("\n========== 四通道 Score 分析 ==========\n");
fprintf("PRESS 阶段：PressScore 90%% = %.2f\n", pressScore_press90);
fprintf("PRESS 阶段：ReleaseScore 干扰 90%% = %.2f\n", releaseScore_press90);
fprintf("\nRELEASE 阶段：ReleaseScore 90%% = %.2f\n", releaseScore_release90);
fprintf("RELEASE 阶段：PressScore 干扰 90%% = %.2f\n", pressScore_release90);
fprintf("\nHOLD 阶段：PressScore 90%% = %.2f\n", pressScore_hold90);
fprintf("HOLD 阶段：ReleaseScore 90%% = %.2f\n", releaseScore_hold90);

fprintf("\n建议判断逻辑：\n");
fprintf("Press intent:   PressScore > 0.55 且 PressScore > 1.4 × ReleaseScore\n");
fprintf("Release intent: ReleaseScore > 0.55 且 ReleaseScore > 1.4 × PressScore\n");
fprintf("======================================\n");

%% ========== 保存汇总文件 ==========
fid2 = fopen(SUMMARY_FILE, "w");
fprintf(fid2, "Metric,Value\n");
fprintf(fid2, "PressScore_PRESS_90,%.3f\n", pressScore_press90);
fprintf(fid2, "ReleaseScore_PRESS_Crosstalk_90,%.3f\n", releaseScore_press90);
fprintf(fid2, "ReleaseScore_RELEASE_90,%.3f\n", releaseScore_release90);
fprintf(fid2, "PressScore_RELEASE_Crosstalk_90,%.3f\n", pressScore_release90);
fprintf(fid2, "PressScore_HOLD_90,%.3f\n", pressScore_hold90);
fprintf(fid2, "ReleaseScore_HOLD_90,%.3f\n", releaseScore_hold90);
fclose(fid2);

disp("汇总文件已保存：");
disp(SUMMARY_FILE);

disp("全部完成。");

%% ========== 本地函数 ==========
function runPhase(s, fid, cmd, durationSec, phaseName, actionText)
    fprintf("\n【%s】%.1f 秒\n", phaseName, durationSec);
    fprintf("动作：%s\n", actionText);

    flush(s);
    writeline(s, char(cmd));
    pause(0.08);
    beep;

    tStart = tic;

    while toc(tStart) < durationSec
        if s.NumBytesAvailable > 0
            line = strtrim(readline(s));

            if strlength(line) == 0
                continue;
            end

            if startsWith(line, "#")
                continue;
            end

            if startsWith(line, "time_ms")
                continue;
            end

            if ~isempty(regexp(line, "^\d+,", "once"))
                fprintf(fid, "%s\n", line);
            end
        end

        pause(0.002);
    end

    disp("完成");
end

function markPhaseChanges(t, labels)
    if isempty(t) || isempty(labels)
        return;
    end

    yl = ylim;
    changeIdx = [1; find(labels(2:end) ~= labels(1:end-1)) + 1];

    for i = 1:numel(changeIdx)
        idx = changeIdx(i);
        x = t(idx);

        xline(x, "k:", "HandleVisibility", "off");

        text(x, yl(2), char(labels(idx)), ...
            "Rotation", 90, ...
            "VerticalAlignment", "top", ...
            "HorizontalAlignment", "right", ...
            "FontSize", 7, ...
            "HandleVisibility", "off");
    end

    ylim(yl);
end

function y = percentileLocal(x, p)
    x = x(~isnan(x));

    if isempty(x)
        y = NaN;
        return;
    end

    x = sort(x);
    n = numel(x);

    if n == 1
        y = x(1);
        return;
    end

    rank = 1 + (p / 100) * (n - 1);
    low = floor(rank);
    high = ceil(rank);

    if low == high
        y = x(low);
    else
        weight = rank - low;
        y = x(low) * (1 - weight) + x(high) * weight;
    end
end