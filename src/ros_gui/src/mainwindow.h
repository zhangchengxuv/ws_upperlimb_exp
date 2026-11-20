#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMutex>
#include <QProgressBar>
#include <QPushButton>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>

#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "upperlimb_robot/msg/control_command.hpp"
#include "upperlimb_robot/msg/experiment_config.hpp"
#include "upperlimb_robot/msg/logger_command.hpp"
#include "upperlimb_robot/msg/robot_state.hpp"
#include "upperlimb_robot/msg/system_state.hpp"

// ================================
// 控制模式枚举（与控制端编号严格一致）
// ================================
enum ControlMode : int
{
    MODE_IDLE = 0,
    MODE_LEFT_PRESET_POSITION = 1,
    MODE_RIGHT_PRESET_POSITION = 2,
    MODE_ACTIVE = 3,
    MODE_ZERO_FORCE = 4,
    MODE_PASSIVE = 5,
    MODE_PASSIVE_SINE_PD_FF = 6,
    MODE_RIGHT_ZERO_FORCE = 8,
    MODE_BILATERAL_ZERO_FORCE = 9,
    MODE_BILATERAL_ACTIVE = 13,
    MODE_RIGHT_ACTIVE = 14,
    MODE_BILATERAL_PRESET_POSITION = 20,
    MODE_LEFT_ADAPTIVE_ACTIVE = 21,
    MODE_RIGHT_ADAPTIVE_ACTIVE = 22,
    MODE_BILATERAL_ADAPTIVE_ACTIVE = 23,
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // 模式按钮槽函数
    void onIdleClicked();
    void onLeftPresetClicked();
    void onRightPresetClicked();
    void onBilateralPresetClicked();
    void onActiveClicked();
    void onRightActiveClicked();
    void onBilateralActiveClicked();
    void onLeftAdaptiveActiveClicked();
    void onRightAdaptiveActiveClicked();
    void onBilateralAdaptiveActiveClicked();
    void onZeroForceClicked();
    void onRightZeroForceClicked();
    void onBilateralZeroForceClicked();
    void onLeftPassivePID();
    void onLeftPassivePD();

    // 实验参数变更（仅空闲态有效）
    void onExperimentConfigChanged();

    // CSV 记录与事件打标
    void onStartRecordClicked();
    void onStopRecordClicked();
    void onEventTagClicked(const QString &tag);

    // 视觉节拍引导条
    void onTogglePacingClicked();
    void updatePacingBar();

    // 10 Hz UI 刷新
    void refreshUi();

private:
    void systemStateCallback(const upperlimb_robot::msg::SystemState::SharedPtr msg);
    void robotStateCallback(const upperlimb_robot::msg::RobotState::SharedPtr msg);

    void publishCommand(int cmd);
    void publishExperimentConfig();
    void publishLoggerCommand(int action, const QString &eventTag = "");
    void appendLog(const QString &text);
    QString modeToString(int mode) const;
    void updateIndicator(QLabel *indicator, QLabel *textLabel, bool ok, const QString &text);

private:
    // ================================
    // ROS2 通信成员
    // ================================
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<upperlimb_robot::msg::ControlCommand>::SharedPtr control_command_pub_;
    rclcpp::Publisher<upperlimb_robot::msg::ExperimentConfig>::SharedPtr experiment_config_pub_;
    rclcpp::Publisher<upperlimb_robot::msg::LoggerCommand>::SharedPtr logger_command_pub_;
    rclcpp::Subscription<upperlimb_robot::msg::SystemState>::SharedPtr system_state_sub_;
    rclcpp::Subscription<upperlimb_robot::msg::RobotState>::SharedPtr robot_state_sub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

    std::thread ros_spin_thread_;

    // ================================
    // 顶部状态栏 UI
    // ================================
    QLabel *controllerInitIndicator_ = nullptr;
    QLabel *controllerInitLabel_ = nullptr;

    QLabel *jointInitIndicator_ = nullptr;
    QLabel *jointInitLabel_ = nullptr;

    QLabel *systemReadyIndicator_ = nullptr;
    QLabel *systemReadyLabel_ = nullptr;

    QLabel *currentModeIndicator_ = nullptr;
    QLabel *currentModeLabel_ = nullptr;

    QLabel *timeLabel_ = nullptr;

    // ================================
    // 左侧控制按钮与参数配置 UI
    // ================================
    QPushButton *idleButton_ = nullptr;
    QPushButton *leftPresetButton_ = nullptr;
    QPushButton *rightPresetButton_ = nullptr;
    QPushButton *bilateralPresetButton_ = nullptr;

    QComboBox *passiveJointCombo_ = nullptr;
    QComboBox *passiveSpeedCombo_ = nullptr;
    QPushButton *leftPassivePIDButton_ = nullptr;
    QPushButton *leftPassivePDButton_ = nullptr;

    QPushButton *zeroForceButton_ = nullptr;
    QPushButton *rightZeroForceButton_ = nullptr;
    QPushButton *bilateralZeroForceButton_ = nullptr;

    QComboBox *admittanceDampingCombo_ = nullptr;
    QPushButton *activeButton_ = nullptr;
    QPushButton *rightActiveButton_ = nullptr;
    QPushButton *bilateralActiveButton_ = nullptr;
    QPushButton *leftAdaptiveActiveButton_ = nullptr;
    QPushButton *rightAdaptiveActiveButton_ = nullptr;
    QPushButton *bilateralAdaptiveActiveButton_ = nullptr;

    QLabel *interlockHintLabel_ = nullptr;

    // ================================
    // 中间实验记录、实时体检与节拍条 UI
    // ================================
    QLabel *actualDtLabel_ = nullptr;
    QLabel *stateElapsedLabel_ = nullptr;
    QLabel *controlElapsedLabel_ = nullptr;
    QLabel *droppedFramesLabel_ = nullptr;

    QLineEdit *subjectEdit_ = nullptr;
    QLineEdit *conditionEdit_ = nullptr;
    QLineEdit *trialEdit_ = nullptr;
    QLabel *recordStatusLabel_ = nullptr;
    QPushButton *startRecordButton_ = nullptr;
    QPushButton *stopRecordButton_ = nullptr;

    QPushButton *tagStartButton_ = nullptr;
    QPushButton *tagHoldButton_ = nullptr;
    QPushButton *tagReleaseButton_ = nullptr;
    QPushButton *tagEndButton_ = nullptr;

    QDoubleSpinBox *pacingPeriodSpin_ = nullptr;
    QPushButton *pacingToggleButton_ = nullptr;
    QProgressBar *pacingBar_ = nullptr;
    QLabel *pacingPhaseLabel_ = nullptr;
    QTimer *pacingTimer_ = nullptr;
    QElapsedTimer pacingClock_;
    bool pacingRunning_ = false;

    // ================================
    // 右侧关节表、六维力与日志框 UI
    // ================================
    QTableWidget *leftJointTable_ = nullptr;
    QTableWidget *rightJointTable_ = nullptr;

    QLabel *rightHandFxLabel_ = nullptr;
    QLabel *rightHandFyLabel_ = nullptr;
    QLabel *rightHandFzLabel_ = nullptr;
    QLabel *rightHandMxLabel_ = nullptr;
    QLabel *rightHandMyLabel_ = nullptr;
    QLabel *rightHandMzLabel_ = nullptr;

    QLabel *rightArmFxLabel_ = nullptr;
    QLabel *rightArmFyLabel_ = nullptr;
    QLabel *rightArmFzLabel_ = nullptr;
    QLabel *rightArmMxLabel_ = nullptr;
    QLabel *rightArmMyLabel_ = nullptr;
    QLabel *rightArmMzLabel_ = nullptr;

    QLabel *leftHandFxLabel_ = nullptr;
    QLabel *leftHandFyLabel_ = nullptr;
    QLabel *leftHandFzLabel_ = nullptr;
    QLabel *leftHandMxLabel_ = nullptr;
    QLabel *leftHandMyLabel_ = nullptr;
    QLabel *leftHandMzLabel_ = nullptr;

    QLabel *leftArmFxLabel_ = nullptr;
    QLabel *leftArmFyLabel_ = nullptr;
    QLabel *leftArmFzLabel_ = nullptr;
    QLabel *leftArmMxLabel_ = nullptr;
    QLabel *leftArmMyLabel_ = nullptr;
    QLabel *leftArmMzLabel_ = nullptr;

    QTextEdit *logTextEdit_ = nullptr;

    QTimer *uiTimer_ = nullptr;

    // ================================
    // 共享缓存（ROS 线程写，Qt 线程读）
    // ================================
    QMutex dataMutex_;

    bool controller_init_ = false;
    bool joint_init_ = false;
    bool system_ready_ = false;
    bool logging_active_ = false;
    std::string current_log_file_;
    std::uint64_t dropped_frames_ = 0;

    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> torque_;
    std::vector<bool> enabled_;
    std::vector<double> force_sensor_;

    double actual_dt_ms_ = 0.0;
    double state_elapsed_ms_ = 0.0;
    double control_elapsed_ms_ = 0.0;

    int requested_mode_ = MODE_IDLE;
    int applied_mode_ = MODE_IDLE;
};

#endif // MAINWINDOW_H
