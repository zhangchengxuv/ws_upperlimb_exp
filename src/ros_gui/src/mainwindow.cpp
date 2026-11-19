#include "mainwindow.h"

#include <QButtonGroup>
#include <QDateTime>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMutexLocker>
#include <QSplitter>
#include <QStyle>
#include <QVBoxLayout>
#include <QWidget>

#include <cmath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // ================================
    // ROS2 初始化
    // ================================
    node_ = rclcpp::Node::make_shared("qt_gui_node");

    control_command_pub_ =
        node_->create_publisher<upperlimb_robot::msg::ControlCommand>("control_command", 10);

    experiment_config_pub_ =
        node_->create_publisher<upperlimb_robot::msg::ExperimentConfig>("experiment_config", 10);

    logger_command_pub_ =
        node_->create_publisher<upperlimb_robot::msg::LoggerCommand>("logger_command", 10);

    system_state_sub_ =
        node_->create_subscription<upperlimb_robot::msg::SystemState>(
            "system_state",
            10,
            std::bind(&MainWindow::systemStateCallback, this, std::placeholders::_1));

    robot_state_sub_ =
        node_->create_subscription<upperlimb_robot::msg::RobotState>(
            "robot_state",
            10,
            std::bind(&MainWindow::robotStateCallback, this, std::placeholders::_1));

    // ================================
    // 主窗口布局
    // ================================
    QWidget *central = new QWidget(this);
    setCentralWidget(central);

    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(8, 8, 8, 8);
    mainLayout->setSpacing(8);

    // ================================
    // 顶部状态栏
    // ================================
    QGroupBox *statusGroup = new QGroupBox("系统状态", central);
    statusGroup->setObjectName("statusPanel");
    QHBoxLayout *statusLayout = new QHBoxLayout(statusGroup);
    statusLayout->setContentsMargins(12, 8, 12, 8);
    statusLayout->setSpacing(12);

    controllerInitIndicator_ = new QLabel(statusGroup);
    controllerInitIndicator_->setObjectName("statusIndicator");
    controllerInitIndicator_->setFixedSize(12, 12);
    controllerInitLabel_ = new QLabel("控制器初始化", statusGroup);
    controllerInitLabel_->setProperty("role", "statusText");

    jointInitIndicator_ = new QLabel(statusGroup);
    jointInitIndicator_->setObjectName("statusIndicator");
    jointInitIndicator_->setFixedSize(12, 12);
    jointInitLabel_ = new QLabel("电机初始化", statusGroup);
    jointInitLabel_->setProperty("role", "statusText");

    systemReadyIndicator_ = new QLabel(statusGroup);
    systemReadyIndicator_->setObjectName("statusIndicator");
    systemReadyIndicator_->setFixedSize(12, 12);
    systemReadyLabel_ = new QLabel("系统就绪", statusGroup);
    systemReadyLabel_->setProperty("role", "statusText");

    currentModeIndicator_ = new QLabel(statusGroup);
    currentModeIndicator_->setObjectName("modeIndicator");
    currentModeIndicator_->setFixedSize(12, 12);
    currentModeLabel_ = new QLabel("当前模式: 安全停止 / 空闲 (0)", statusGroup);
    currentModeLabel_->setProperty("role", "modeText");

    timeLabel_ = new QLabel("--", statusGroup);
    timeLabel_->setObjectName("timeLabel");

    controllerInitIndicator_->setProperty("status", "offline");
    jointInitIndicator_->setProperty("status", "offline");
    systemReadyIndicator_->setProperty("status", "offline");
    currentModeIndicator_->setProperty("status", "active");

    statusLayout->addWidget(controllerInitIndicator_);
    statusLayout->addWidget(controllerInitLabel_);
    statusLayout->addSpacing(20);

    statusLayout->addWidget(jointInitIndicator_);
    statusLayout->addWidget(jointInitLabel_);
    statusLayout->addSpacing(20);

    statusLayout->addWidget(systemReadyIndicator_);
    statusLayout->addWidget(systemReadyLabel_);
    statusLayout->addSpacing(20);

    statusLayout->addWidget(currentModeIndicator_);
    statusLayout->addWidget(currentModeLabel_);

    statusLayout->addStretch();
    statusLayout->addWidget(timeLabel_);

    // ================================
    // 主区域三栏分割（左：控制区，中：实验记录与辅助，右：数据+日志）
    // ================================
    QSplitter *mainSplitter = new QSplitter(Qt::Horizontal, central);
    mainSplitter->setChildrenCollapsible(false);

    // ================================
    // 左侧控制区
    // ================================
    QGroupBox *controlGroup = new QGroupBox("控制模式与安全操作", mainSplitter);
    controlGroup->setObjectName("sidebarPanel");
    QVBoxLayout *controlLayout = new QVBoxLayout(controlGroup);
    controlLayout->setContentsMargins(10, 12, 10, 10);
    controlLayout->setSpacing(6);

    auto addSectionTitle = [controlGroup, controlLayout](const QString &text)
    {
        QLabel *label = new QLabel(text, controlGroup);
        label->setProperty("role", "sectionTitle");
        controlLayout->addWidget(label);
    };

    // 1. 醒目的安全停止 / 空闲按钮
    idleButton_ = new QPushButton("安全停止 / 空闲 (MODE=0)", controlGroup);
    idleButton_->setObjectName("safetyStopButton");

    // 2. 恢复初始位置按钮（原样保留）
    leftPresetButton_ = new QPushButton("左臂恢复初始位置 (MODE=1)", controlGroup);
    rightPresetButton_ = new QPushButton("右臂恢复初始位置 (MODE=2)", controlGroup);

    // 3. 被动正弦实验按钮与选轴/低速配置
    passiveJointCombo_ = new QComboBox(controlGroup);
    passiveJointCombo_->addItem("单关节 EB1 (肘屈伸)", 4);
    passiveJointCombo_->addItem("单关节 GH1 (肩外展内收)", 1);
    passiveJointCombo_->addItem("单关节 GH3 (肩前屈后伸)", 3);
    passiveJointCombo_->addItem("单关节 GH2 (肩内旋外旋)", 2);
    passiveJointCombo_->addItem("组合联动 GH1~GH3 + EB1", 0);
    passiveJointCombo_->setCurrentIndex(4); // 默认组合联动，可一键切单关节

    passiveSpeedCombo_ = new QComboBox(controlGroup);
    passiveSpeedCombo_->addItem("低速 1 档 (慢速 0.5x)", 0);
    passiveSpeedCombo_->addItem("低速 2 档 (标准 1.0x)", 1);
    passiveSpeedCombo_->setCurrentIndex(1);

    leftPassivePIDButton_ = new QPushButton("被动正弦 PID (MODE=5)", controlGroup);
    leftPassivePDButton_ = new QPushButton("被动正弦 PD+FF (MODE=6)", controlGroup);

    // 4. 零力拖动按钮
    zeroForceButton_ = new QPushButton("左臂零力拖动 (MODE=4)", controlGroup);
    rightZeroForceButton_ = new QPushButton("右臂零力拖动 (MODE=8)", controlGroup);
    bilateralZeroForceButton_ = new QPushButton("双臂零力拖动 (MODE=9)", controlGroup);

    // 5. 主动导纳按钮与预设阻尼档位
    admittanceDampingCombo_ = new QComboBox(controlGroup);
    admittanceDampingCombo_->addItem("低阻尼档 (0.70x 轻快)", 0);
    admittanceDampingCombo_->addItem("标准阻尼档 (1.00x 原参数)", 1);
    admittanceDampingCombo_->addItem("高阻尼档 (1.50x 稳重)", 2);
    admittanceDampingCombo_->setCurrentIndex(1);

    activeButton_ = new QPushButton("左臂主动导纳 (MODE=3)", controlGroup);
    rightActiveButton_ = new QPushButton("右臂主动导纳 (MODE=14)", controlGroup);
    bilateralActiveButton_ = new QPushButton("双侧主动导纳 (MODE=13)", controlGroup);

    interlockHintLabel_ = new QLabel("参数可调（当前空闲态）", controlGroup);
    interlockHintLabel_->setProperty("role", "panelHint");
    interlockHintLabel_->setWordWrap(true);

    const QList<QPushButton *> modeButtons = {
        idleButton_, leftPresetButton_, rightPresetButton_,
        leftPassivePIDButton_, leftPassivePDButton_,
        zeroForceButton_, rightZeroForceButton_, bilateralZeroForceButton_,
        activeButton_, rightActiveButton_, bilateralActiveButton_};

    QButtonGroup *modeGroup = new QButtonGroup(this);
    modeGroup->setExclusive(true);
    for (QPushButton *btn : modeButtons)
    {
        btn->setCursor(Qt::PointingHandCursor);
        btn->setCheckable(true);
        modeGroup->addButton(btn);
    }
    idleButton_->setChecked(true);

    connect(idleButton_, &QPushButton::clicked, this, &MainWindow::onIdleClicked);
    connect(leftPresetButton_, &QPushButton::clicked, this, &MainWindow::onLeftPresetClicked);
    connect(rightPresetButton_, &QPushButton::clicked, this, &MainWindow::onRightPresetClicked);
    connect(leftPassivePIDButton_, &QPushButton::clicked, this, &MainWindow::onLeftPassivePID);
    connect(leftPassivePDButton_, &QPushButton::clicked, this, &MainWindow::onLeftPassivePD);
    connect(zeroForceButton_, &QPushButton::clicked, this, &MainWindow::onZeroForceClicked);
    connect(rightZeroForceButton_, &QPushButton::clicked, this, &MainWindow::onRightZeroForceClicked);
    connect(bilateralZeroForceButton_, &QPushButton::clicked, this, &MainWindow::onBilateralZeroForceClicked);
    connect(activeButton_, &QPushButton::clicked, this, &MainWindow::onActiveClicked);
    connect(rightActiveButton_, &QPushButton::clicked, this, &MainWindow::onRightActiveClicked);
    connect(bilateralActiveButton_, &QPushButton::clicked, this, &MainWindow::onBilateralActiveClicked);

    connect(passiveJointCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onExperimentConfigChanged);
    connect(passiveSpeedCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onExperimentConfigChanged);
    connect(admittanceDampingCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onExperimentConfigChanged);

    controlLayout->addWidget(idleButton_);

    addSectionTitle("恢复初始位置");
    controlLayout->addWidget(leftPresetButton_);
    controlLayout->addWidget(rightPresetButton_);

    addSectionTitle("实验 1：被动正弦跟踪（左臂）");
    controlLayout->addWidget(passiveJointCombo_);
    controlLayout->addWidget(passiveSpeedCombo_);
    controlLayout->addWidget(leftPassivePIDButton_);
    controlLayout->addWidget(leftPassivePDButton_);

    addSectionTitle("实验 2：零力拖动");
    controlLayout->addWidget(zeroForceButton_);
    controlLayout->addWidget(rightZeroForceButton_);
    controlLayout->addWidget(bilateralZeroForceButton_);

    addSectionTitle("实验 3：主动导纳");
    controlLayout->addWidget(admittanceDampingCombo_);
    controlLayout->addWidget(activeButton_);
    controlLayout->addWidget(rightActiveButton_);
    controlLayout->addWidget(bilateralActiveButton_);

    controlLayout->addWidget(interlockHintLabel_);
    controlLayout->addStretch();

    // ================================
    // 中间栏：实时性体检 + 异步 CSV 记录 + 视觉节拍引导条
    // ================================
    QGroupBox *expHelperGroup = new QGroupBox("实验记录与辅助", mainSplitter);
    expHelperGroup->setObjectName("experimentPanel");
    QVBoxLayout *helperLayout = new QVBoxLayout(expHelperGroup);
    helperLayout->setContentsMargins(10, 12, 10, 10);
    helperLayout->setSpacing(10);

    // 1) 实时性体检区
    QGroupBox *diagGroup = new QGroupBox("实时性体检", expHelperGroup);
    diagGroup->setObjectName("dataPanel");
    QVBoxLayout *diagLayout = new QVBoxLayout(diagGroup);
    diagLayout->setContentsMargins(10, 8, 10, 8);
    diagLayout->setSpacing(4);

    actualDtLabel_ = new QLabel("actual_dt: -- ms", diagGroup);
    stateElapsedLabel_ = new QLabel("state_elapsed: -- ms", diagGroup);
    controlElapsedLabel_ = new QLabel("control_elapsed: -- ms", diagGroup);
    droppedFramesLabel_ = new QLabel("dropped_frames: 0", diagGroup);

    for (QLabel *lbl : {actualDtLabel_, stateElapsedLabel_, controlElapsedLabel_, droppedFramesLabel_})
    {
        lbl->setProperty("role", "metric");
        diagLayout->addWidget(lbl);
    }

    // 2) 异步 CSV 记录与事件打标区
    QGroupBox *recGroup = new QGroupBox("CSV 记录与事件打标", expHelperGroup);
    recGroup->setObjectName("dataPanel");
    QVBoxLayout *recLayout = new QVBoxLayout(recGroup);
    recLayout->setContentsMargins(10, 8, 10, 8);
    recLayout->setSpacing(8);

    QFormLayout *metaForm = new QFormLayout();
    metaForm->setSpacing(6);
    subjectEdit_ = new QLineEdit("S01", recGroup);
    conditionEdit_ = new QLineEdit("exp_base", recGroup);
    trialEdit_ = new QLineEdit("T01", recGroup);
    metaForm->addRow("subject_id:", subjectEdit_);
    metaForm->addRow("condition:", conditionEdit_);
    metaForm->addRow("trial_id:", trialEdit_);
    recLayout->addLayout(metaForm);

    recordStatusLabel_ = new QLabel("记录状态: 未记录", recGroup);
    recordStatusLabel_->setProperty("role", "panelHint");
    recordStatusLabel_->setWordWrap(true);
    recLayout->addWidget(recordStatusLabel_);

    QHBoxLayout *recBtnLayout = new QHBoxLayout();
    startRecordButton_ = new QPushButton("开始记录", recGroup);
    startRecordButton_->setObjectName("recordStartButton");
    startRecordButton_->setCursor(Qt::PointingHandCursor);

    stopRecordButton_ = new QPushButton("停止记录", recGroup);
    stopRecordButton_->setObjectName("recordStopButton");
    stopRecordButton_->setCursor(Qt::PointingHandCursor);

    recBtnLayout->addWidget(startRecordButton_);
    recBtnLayout->addWidget(stopRecordButton_);
    recLayout->addLayout(recBtnLayout);

    QLabel *tagHint = new QLabel("一键事件打标（写入 event_tag 列）:", recGroup);
    tagHint->setProperty("role", "panelHint");
    recLayout->addWidget(tagHint);

    QGridLayout *tagGrid = new QGridLayout();
    tagGrid->setSpacing(6);
    tagStartButton_ = new QPushButton("开始 (START)", recGroup);
    tagHoldButton_ = new QPushButton("保持 (HOLD)", recGroup);
    tagReleaseButton_ = new QPushButton("释放 (RELEASE)", recGroup);
    tagEndButton_ = new QPushButton("结束 (END)", recGroup);

    for (QPushButton *tb : {tagStartButton_, tagHoldButton_, tagReleaseButton_, tagEndButton_})
    {
        tb->setProperty("role", "tagButton");
        tb->setCursor(Qt::PointingHandCursor);
    }

    tagGrid->addWidget(tagStartButton_, 0, 0);
    tagGrid->addWidget(tagHoldButton_, 0, 1);
    tagGrid->addWidget(tagReleaseButton_, 1, 0);
    tagGrid->addWidget(tagEndButton_, 1, 1);
    recLayout->addLayout(tagGrid);

    connect(startRecordButton_, &QPushButton::clicked, this, &MainWindow::onStartRecordClicked);
    connect(stopRecordButton_, &QPushButton::clicked, this, &MainWindow::onStopRecordClicked);
    connect(tagStartButton_, &QPushButton::clicked, this, [this]() { onEventTagClicked("START"); });
    connect(tagHoldButton_, &QPushButton::clicked, this, [this]() { onEventTagClicked("HOLD"); });
    connect(tagReleaseButton_, &QPushButton::clicked, this, [this]() { onEventTagClicked("RELEASE"); });
    connect(tagEndButton_, &QPushButton::clicked, this, [this]() { onEventTagClicked("END"); });

    // 3) 视觉节拍引导条（辅助零力与导纳拖动保持相近速度）
    QGroupBox *pacingGroup = new QGroupBox("视觉节拍引导条", expHelperGroup);
    pacingGroup->setObjectName("dataPanel");
    QVBoxLayout *pacingLayout = new QVBoxLayout(pacingGroup);
    pacingLayout->setContentsMargins(10, 8, 10, 8);
    pacingLayout->setSpacing(6);

    QHBoxLayout *pacingCtrlLayout = new QHBoxLayout();
    QLabel *periodLbl = new QLabel("往返周期:", pacingGroup);
    pacingPeriodSpin_ = new QDoubleSpinBox(pacingGroup);
    pacingPeriodSpin_->setRange(2.0, 20.0);
    pacingPeriodSpin_->setSingleStep(0.5);
    pacingPeriodSpin_->setDecimals(1);
    pacingPeriodSpin_->setValue(6.0);
    pacingPeriodSpin_->setSuffix(" s");

    pacingToggleButton_ = new QPushButton("启动节拍", pacingGroup);
    pacingToggleButton_->setCursor(Qt::PointingHandCursor);

    pacingCtrlLayout->addWidget(periodLbl);
    pacingCtrlLayout->addWidget(pacingPeriodSpin_);
    pacingCtrlLayout->addWidget(pacingToggleButton_);
    pacingLayout->addLayout(pacingCtrlLayout);

    pacingBar_ = new QProgressBar(pacingGroup);
    pacingBar_->setRange(0, 1000);
    pacingBar_->setValue(0);
    pacingBar_->setFormat("%p%");
    pacingLayout->addWidget(pacingBar_);

    pacingPhaseLabel_ = new QLabel("节拍状态: 已停止", pacingGroup);
    pacingPhaseLabel_->setProperty("role", "panelHint");
    pacingLayout->addWidget(pacingPhaseLabel_);

    connect(pacingToggleButton_, &QPushButton::clicked, this, &MainWindow::onTogglePacingClicked);

    pacingTimer_ = new QTimer(this);
    connect(pacingTimer_, &QTimer::timeout, this, &MainWindow::updatePacingBar);

    helperLayout->addWidget(diagGroup);
    helperLayout->addWidget(recGroup);
    helperLayout->addWidget(pacingGroup);
    helperLayout->addStretch();

    // ================================
    // 右侧上下分区（数据区 + 日志框）
    // ================================
    QSplitter *rightSplitter = new QSplitter(Qt::Vertical, mainSplitter);
    rightSplitter->setChildrenCollapsible(false);

    QWidget *dataWidget = new QWidget(rightSplitter);
    QVBoxLayout *dataMainLayout = new QVBoxLayout(dataWidget);
    dataMainLayout->setContentsMargins(0, 0, 0, 0);
    dataMainLayout->setSpacing(8);

    // 关节状态表（同时标注论文代号与功能）
    QGroupBox *jointGroup = new QGroupBox("关节状态（前7轴活动控制，WR1~WR3当前未启用）", dataWidget);
    jointGroup->setObjectName("dataPanel");
    QHBoxLayout *jointLayout = new QHBoxLayout(jointGroup);
    jointLayout->setContentsMargins(8, 8, 8, 8);
    jointLayout->setSpacing(8);

    const QStringList jointFuncs = {
        "SG1(肩带升降)",
        "SG2(肩带前后)",
        "SG3(肩带姿态)",
        "GH1(外展内收)",
        "GH2(内旋外旋)",
        "GH3(前屈后伸)",
        "EB1(肘屈伸)",
        "WR1(旋前旋后-未用)",
        "WR2(尺偏桡偏-未用)",
        "WR3(腕屈伸-未用)"};

    auto createJointTable = [&jointFuncs](QWidget *parent, const QString &sidePrefix) -> QTableWidget *
    {
        QTableWidget *table = new QTableWidget(parent);
        table->setColumnCount(5);
        table->setHorizontalHeaderLabels(
            QStringList() << "关节(功能)" << "位置(deg)" << "速度(deg/s)" << "力矩(Nm)" << "使能");
        table->setRowCount(10);

        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
        table->setColumnWidth(0, 145);

        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
        table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);

        table->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed);
        table->setColumnWidth(4, 52);

        table->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);
        table->verticalHeader()->setVisible(false);
        table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table->setSelectionMode(QAbstractItemView::NoSelection);
        table->setAlternatingRowColors(true);
        table->setFocusPolicy(Qt::NoFocus);
        table->verticalHeader()->setDefaultSectionSize(28);

        for (int i = 0; i < 10; ++i)
        {
            QTableWidgetItem *axisItem = new QTableWidgetItem(sidePrefix + "_" + jointFuncs[i]);
            axisItem->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
            table->setItem(i, 0, axisItem);

            for (int col = 1; col < 5; ++col)
            {
                QTableWidgetItem *item = new QTableWidgetItem("--");
                item->setTextAlignment(Qt::AlignCenter);
                table->setItem(i, col, item);
            }
        }
        return table;
    };

    QGroupBox *leftJointGroup = new QGroupBox("左臂关节 (joint1~10 / 物理轴11~20)", jointGroup);
    QVBoxLayout *leftJointLayout = new QVBoxLayout(leftJointGroup);
    leftJointLayout->setContentsMargins(6, 6, 6, 6);

    QGroupBox *rightJointGroup = new QGroupBox("右臂关节 (joint11~20 / 物理轴1~10)", jointGroup);
    QVBoxLayout *rightJointLayout = new QVBoxLayout(rightJointGroup);
    rightJointLayout->setContentsMargins(6, 6, 6, 6);

    leftJointTable_ = createJointTable(leftJointGroup, "L");
    rightJointTable_ = createJointTable(rightJointGroup, "R");

    leftJointLayout->addWidget(leftJointTable_);
    rightJointLayout->addWidget(rightJointTable_);

    jointLayout->addWidget(leftJointGroup);
    jointLayout->addWidget(rightJointGroup);

    // 四处六维力显示区（已移除旧虚拟弹簧显示区）
    QGroupBox *forceGroup = new QGroupBox("四处六维力传感器信息（单位：N / Nm）", dataWidget);
    forceGroup->setObjectName("dataPanel");
    QGridLayout *forceLayout = new QGridLayout(forceGroup);
    forceLayout->setContentsMargins(12, 8, 12, 8);
    forceLayout->setHorizontalSpacing(18);
    forceLayout->setVerticalSpacing(5);

    QLabel *rightHandTitle = new QLabel("右手传感器 (RH)", forceGroup);
    QLabel *rightArmTitle = new QLabel("右臂传感器 (RA)", forceGroup);
    QLabel *leftHandTitle = new QLabel("左手传感器 (LH)", forceGroup);
    QLabel *leftArmTitle = new QLabel("左臂传感器 (LA)", forceGroup);

    for (QLabel *title : {rightHandTitle, rightArmTitle, leftHandTitle, leftArmTitle})
    {
        title->setProperty("role", "cardTitle");
    }

    rightHandFxLabel_ = new QLabel("Fx: --", forceGroup);
    rightHandFyLabel_ = new QLabel("Fy: --", forceGroup);
    rightHandFzLabel_ = new QLabel("Fz: --", forceGroup);
    rightHandMxLabel_ = new QLabel("Mx: --", forceGroup);
    rightHandMyLabel_ = new QLabel("My: --", forceGroup);
    rightHandMzLabel_ = new QLabel("Mz: --", forceGroup);

    rightArmFxLabel_ = new QLabel("Fx: --", forceGroup);
    rightArmFyLabel_ = new QLabel("Fy: --", forceGroup);
    rightArmFzLabel_ = new QLabel("Fz: --", forceGroup);
    rightArmMxLabel_ = new QLabel("Mx: --", forceGroup);
    rightArmMyLabel_ = new QLabel("My: --", forceGroup);
    rightArmMzLabel_ = new QLabel("Mz: --", forceGroup);

    leftHandFxLabel_ = new QLabel("Fx: --", forceGroup);
    leftHandFyLabel_ = new QLabel("Fy: --", forceGroup);
    leftHandFzLabel_ = new QLabel("Fz: --", forceGroup);
    leftHandMxLabel_ = new QLabel("Mx: --", forceGroup);
    leftHandMyLabel_ = new QLabel("My: --", forceGroup);
    leftHandMzLabel_ = new QLabel("Mz: --", forceGroup);

    leftArmFxLabel_ = new QLabel("Fx: --", forceGroup);
    leftArmFyLabel_ = new QLabel("Fy: --", forceGroup);
    leftArmFzLabel_ = new QLabel("Fz: --", forceGroup);
    leftArmMxLabel_ = new QLabel("Mx: --", forceGroup);
    leftArmMyLabel_ = new QLabel("My: --", forceGroup);
    leftArmMzLabel_ = new QLabel("Mz: --", forceGroup);

    const QList<QLabel *> forceValueLabels = {
        rightHandFxLabel_, rightHandFyLabel_, rightHandFzLabel_, rightHandMxLabel_, rightHandMyLabel_, rightHandMzLabel_,
        rightArmFxLabel_, rightArmFyLabel_, rightArmFzLabel_, rightArmMxLabel_, rightArmMyLabel_, rightArmMzLabel_,
        leftHandFxLabel_, leftHandFyLabel_, leftHandFzLabel_, leftHandMxLabel_, leftHandMyLabel_, leftHandMzLabel_,
        leftArmFxLabel_, leftArmFyLabel_, leftArmFzLabel_, leftArmMxLabel_, leftArmMyLabel_, leftArmMzLabel_};
    for (QLabel *label : forceValueLabels)
    {
        label->setProperty("role", "metric");
    }

    forceLayout->addWidget(rightHandTitle, 0, 0, 1, 2);
    forceLayout->addWidget(rightHandFxLabel_, 1, 0);
    forceLayout->addWidget(rightHandFyLabel_, 1, 1);
    forceLayout->addWidget(rightHandFzLabel_, 2, 0);
    forceLayout->addWidget(rightHandMxLabel_, 2, 1);
    forceLayout->addWidget(rightHandMyLabel_, 3, 0);
    forceLayout->addWidget(rightHandMzLabel_, 3, 1);

    forceLayout->addWidget(rightArmTitle, 0, 2, 1, 2);
    forceLayout->addWidget(rightArmFxLabel_, 1, 2);
    forceLayout->addWidget(rightArmFyLabel_, 1, 3);
    forceLayout->addWidget(rightArmFzLabel_, 2, 2);
    forceLayout->addWidget(rightArmMxLabel_, 2, 3);
    forceLayout->addWidget(rightArmMyLabel_, 3, 2);
    forceLayout->addWidget(rightArmMzLabel_, 3, 3);

    forceLayout->addWidget(leftHandTitle, 4, 0, 1, 2);
    forceLayout->addWidget(leftHandFxLabel_, 5, 0);
    forceLayout->addWidget(leftHandFyLabel_, 5, 1);
    forceLayout->addWidget(leftHandFzLabel_, 6, 0);
    forceLayout->addWidget(leftHandMxLabel_, 6, 1);
    forceLayout->addWidget(leftHandMyLabel_, 7, 0);
    forceLayout->addWidget(leftHandMzLabel_, 7, 1);

    forceLayout->addWidget(leftArmTitle, 4, 2, 1, 2);
    forceLayout->addWidget(leftArmFxLabel_, 5, 2);
    forceLayout->addWidget(leftArmFyLabel_, 5, 3);
    forceLayout->addWidget(leftArmFzLabel_, 6, 2);
    forceLayout->addWidget(leftArmMxLabel_, 6, 3);
    forceLayout->addWidget(leftArmMyLabel_, 7, 2);
    forceLayout->addWidget(leftArmMzLabel_, 7, 3);

    dataMainLayout->addWidget(jointGroup, 3);
    dataMainLayout->addWidget(forceGroup, 1);

    // 运行日志框
    QGroupBox *logGroup = new QGroupBox("运行日志", rightSplitter);
    logGroup->setObjectName("dataPanel");
    QVBoxLayout *logLayout = new QVBoxLayout(logGroup);

    logTextEdit_ = new QTextEdit(logGroup);
    logTextEdit_->setReadOnly(true);
    logLayout->addWidget(logTextEdit_);

    mainSplitter->addWidget(controlGroup);
    mainSplitter->addWidget(expHelperGroup);
    mainSplitter->addWidget(rightSplitter);
    mainSplitter->setStretchFactor(0, 0);
    mainSplitter->setStretchFactor(1, 0);
    mainSplitter->setStretchFactor(2, 1);
    mainSplitter->setSizes(QList<int>() << 255 << 255 << 990);

    rightSplitter->addWidget(dataWidget);
    rightSplitter->addWidget(logGroup);
    rightSplitter->setStretchFactor(0, 7);
    rightSplitter->setStretchFactor(1, 3);

    mainLayout->addWidget(statusGroup);
    mainLayout->addWidget(mainSplitter, 1);

    setWindowTitle("UpperLimb Exoskeleton - 中期基础实验控制台");
    resize(1500, 920);

    // ================================
    // UI 定时刷新 (10 Hz)
    // ================================
    uiTimer_ = new QTimer(this);
    connect(uiTimer_, &QTimer::timeout, this, &MainWindow::refreshUi);
    uiTimer_->start(100);

    // ================================
    // 启动 ROS SingleThreadedExecutor 线程
    // ================================
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    ros_spin_thread_ = std::thread([this]() { executor_->spin(); });

    appendLog("GUI 启动完成（中期基础实验版，默认空闲态）");
}

MainWindow::~MainWindow()
{
    if (executor_)
    {
        executor_->cancel();
        executor_->remove_node(node_);
    }

    if (ros_spin_thread_.joinable())
    {
        ros_spin_thread_.join();
    }
}

void MainWindow::publishCommand(int cmd)
{
    // 若从空闲态切换进入实验模式，先同步一次当前实验参数
    int curApplied = MODE_IDLE;
    {
        QMutexLocker locker(&dataMutex_);
        curApplied = applied_mode_;
    }
    if (curApplied == MODE_IDLE && cmd != MODE_IDLE)
    {
        publishExperimentConfig();
    }

    upperlimb_robot::msg::ControlCommand msg;
    msg.command = cmd;
    control_command_pub_->publish(msg);

    {
        QMutexLocker locker(&dataMutex_);
        requested_mode_ = cmd;
    }

    appendLog(QString("发送控制命令: %1 (MODE=%2)").arg(modeToString(cmd)).arg(cmd));
}

void MainWindow::publishExperimentConfig()
{
    upperlimb_robot::msg::ExperimentConfig msg;
    msg.passive_joint_profile = passiveJointCombo_->currentData().toInt();
    msg.passive_speed_level = passiveSpeedCombo_->currentData().toInt();
    msg.admittance_damping_level = admittanceDampingCombo_->currentData().toInt();
    experiment_config_pub_->publish(msg);
}

void MainWindow::publishLoggerCommand(int action, const QString &eventTag)
{
    upperlimb_robot::msg::LoggerCommand msg;
    msg.action = action;
    msg.subject_id = subjectEdit_->text().trimmed().toStdString();
    msg.condition = conditionEdit_->text().trimmed().toStdString();
    msg.trial_id = trialEdit_->text().trimmed().toStdString();
    msg.event_tag = eventTag.toStdString();
    logger_command_pub_->publish(msg);
}

void MainWindow::onIdleClicked() { publishCommand(MODE_IDLE); }
void MainWindow::onLeftPresetClicked() { publishCommand(MODE_LEFT_PRESET_POSITION); }
void MainWindow::onRightPresetClicked() { publishCommand(MODE_RIGHT_PRESET_POSITION); }
void MainWindow::onActiveClicked() { publishCommand(MODE_ACTIVE); }
void MainWindow::onRightActiveClicked() { publishCommand(MODE_RIGHT_ACTIVE); }
void MainWindow::onBilateralActiveClicked() { publishCommand(MODE_BILATERAL_ACTIVE); }
void MainWindow::onZeroForceClicked() { publishCommand(MODE_ZERO_FORCE); }
void MainWindow::onRightZeroForceClicked() { publishCommand(MODE_RIGHT_ZERO_FORCE); }
void MainWindow::onBilateralZeroForceClicked() { publishCommand(MODE_BILATERAL_ZERO_FORCE); }
void MainWindow::onLeftPassivePID() { publishCommand(MODE_PASSIVE); }
void MainWindow::onLeftPassivePD() { publishCommand(MODE_PASSIVE_SINE_PD_FF); }

void MainWindow::onExperimentConfigChanged()
{
    int curApplied = MODE_IDLE;
    {
        QMutexLocker locker(&dataMutex_);
        curApplied = applied_mode_;
    }

    if (curApplied != MODE_IDLE)
    {
        appendLog("安全互锁：当前非空闲态，忽略参数切换请求");
        return;
    }

    publishExperimentConfig();
    appendLog(QString("更新实验参数: 选轴=%1 | 速度=%2 | 导纳阻尼=%3")
                  .arg(passiveJointCombo_->currentText())
                  .arg(passiveSpeedCombo_->currentText())
                  .arg(admittanceDampingCombo_->currentText()));
}

void MainWindow::onStartRecordClicked()
{
    publishLoggerCommand(1, "REC_START");
    appendLog(QString("请求开始 CSV 记录: subject=%1, condition=%2, trial=%3")
                  .arg(subjectEdit_->text().trimmed())
                  .arg(conditionEdit_->text().trimmed())
                  .arg(trialEdit_->text().trimmed()));
}

void MainWindow::onStopRecordClicked()
{
    publishLoggerCommand(2);
    appendLog("请求停止 CSV 记录并刷盘");
}

void MainWindow::onEventTagClicked(const QString &tag)
{
    publishLoggerCommand(3, tag);
    appendLog(QString("发送事件打标: [%1]").arg(tag));
}

void MainWindow::onTogglePacingClicked()
{
    if (!pacingRunning_)
    {
        pacingRunning_ = true;
        pacingClock_.start();
        pacingTimer_->start(50);
        pacingToggleButton_->setText("停止节拍");
        appendLog(QString("启动视觉节拍引导条（往返周期 %1 s）")
                      .arg(pacingPeriodSpin_->value(), 0, 'f', 1));
    }
    else
    {
        pacingRunning_ = false;
        pacingTimer_->stop();
        pacingBar_->setValue(0);
        pacingPhaseLabel_->setText("节拍状态: 已停止");
        pacingToggleButton_->setText("启动节拍");
        appendLog("停止视觉节拍引导条");
    }
}

void MainWindow::updatePacingBar()
{
    if (!pacingRunning_)
    {
        return;
    }
    const double period_s = std::max(1.0, pacingPeriodSpin_->value());
    const double elapsed_s = static_cast<double>(pacingClock_.elapsed()) / 1000.0;
    const double phase = std::fmod(elapsed_s, period_s) / period_s; // [0, 1)

    // 余弦往返：0 -> 0.5 去程 (0% -> 100%)，0.5 -> 1.0 回程 (100% -> 0%)
    const double val_norm = 0.5 * (1.0 - std::cos(2.0 * M_PI * phase));
    const int bar_val = static_cast<int>(std::lround(val_norm * 1000.0));
    pacingBar_->setValue(bar_val);

    if (phase < 0.5)
    {
        pacingPhaseLabel_->setText(
            QString("节拍状态: 去程 → (%1 / %2 s)")
                .arg(phase * period_s, 0, 'f', 1)
                .arg(period_s, 0, 'f', 1));
    }
    else
    {
        pacingPhaseLabel_->setText(
            QString("节拍状态: 回程 ← (%1 / %2 s)")
                .arg(phase * period_s, 0, 'f', 1)
                .arg(period_s, 0, 'f', 1));
    }
}

void MainWindow::systemStateCallback(const upperlimb_robot::msg::SystemState::SharedPtr msg)
{
    QMutexLocker locker(&dataMutex_);
    controller_init_ = msg->controller_init;
    joint_init_ = msg->joint_init;
    system_ready_ = msg->system_ready;
    applied_mode_ = msg->applied_mode;
    logging_active_ = msg->logging_active;
    current_log_file_ = msg->current_log_file;
    dropped_frames_ = msg->dropped_frames;
}

void MainWindow::robotStateCallback(const upperlimb_robot::msg::RobotState::SharedPtr msg)
{
    QMutexLocker locker(&dataMutex_);
    position_ = msg->position;
    velocity_ = msg->velocity;
    torque_ = msg->torque;
    enabled_ = msg->enabled;
    force_sensor_ = msg->force_sensor;
    actual_dt_ms_ = msg->actual_dt_ms;
    state_elapsed_ms_ = msg->state_elapsed_ms;
    control_elapsed_ms_ = msg->control_elapsed_ms;
    applied_mode_ = msg->applied_mode;
}

void MainWindow::refreshUi()
{
    QMutexLocker locker(&dataMutex_);

    updateIndicator(controllerInitIndicator_, controllerInitLabel_, controller_init_, "控制器初始化");
    updateIndicator(jointInitIndicator_, jointInitLabel_, joint_init_, "电机初始化");
    updateIndicator(systemReadyIndicator_, systemReadyLabel_, system_ready_, "系统就绪");

    currentModeLabel_->setText(
        QString("当前模式: %1 (生效=%2 / 请求=%3)")
            .arg(modeToString(applied_mode_))
            .arg(applied_mode_)
            .arg(requested_mode_));

    timeLabel_->setText(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));

    // 安全互锁：非 IDLE 时锁定参数下拉框，防止运行中误触导致参数突变
    const bool allow_config = (applied_mode_ == MODE_IDLE);
    passiveJointCombo_->setEnabled(allow_config);
    passiveSpeedCombo_->setEnabled(allow_config);
    admittanceDampingCombo_->setEnabled(allow_config);
    interlockHintLabel_->setText(
        allow_config
            ? "参数互锁：当前空闲态，可切换实验参数"
            : "参数互锁：运行中已锁定参数（请先按安全停止 MODE=0）");

    // 实时性体检显示
    actualDtLabel_->setText(QString("actual_dt: %1 ms").arg(actual_dt_ms_, 0, 'f', 2));
    stateElapsedLabel_->setText(QString("state_elapsed: %1 ms").arg(state_elapsed_ms_, 0, 'f', 2));
    controlElapsedLabel_->setText(QString("control_elapsed: %1 ms").arg(control_elapsed_ms_, 0, 'f', 2));
    droppedFramesLabel_->setText(QString("dropped_frames: %1").arg(static_cast<qulonglong>(dropped_frames_)));

    // 记录状态显示
    if (logging_active_)
    {
        recordStatusLabel_->setText(
            QString("记录状态: ● 记录中 (%1)").arg(QString::fromStdString(current_log_file_)));
        recordStatusLabel_->setStyleSheet("QLabel { color: #4cd17b; font-weight: 600; }");
    }
    else
    {
        recordStatusLabel_->setText("记录状态: 未记录");
        recordStatusLabel_->setStyleSheet("QLabel { color: #8494a8; }");
    }

    // 关节状态表刷新（左臂: 10~19, 右臂: 0~9，保持原有控制层索引映射不变）
    auto updateJointTable = [this](QTableWidget *table, int offset)
    {
        for (int row = 0; row < 10; ++row)
        {
            const int idx = offset + row;

            QString posText = "--";
            QString velText = "--";
            QString torText = "--";
            QString enText = "--";

            if (idx >= 0 && idx < static_cast<int>(position_.size()))
            {
                posText = QString::number(position_[idx], 'f', 2);
            }
            if (idx >= 0 && idx < static_cast<int>(velocity_.size()))
            {
                velText = QString::number(velocity_[idx], 'f', 2);
            }
            if (idx >= 0 && idx < static_cast<int>(torque_.size()))
            {
                torText = QString::number(torque_[idx], 'f', 3);
            }
            if (idx >= 0 && idx < static_cast<int>(enabled_.size()))
            {
                enText = enabled_[idx] ? "ON" : "OFF";
            }

            table->item(row, 1)->setText(posText);
            table->item(row, 2)->setText(velText);
            table->item(row, 3)->setText(torText);
            table->item(row, 4)->setText(enText);
        }
    };

    updateJointTable(leftJointTable_, 10);
    updateJointTable(rightJointTable_, 0);

    // 四处六维力显示（0~5 右手, 6~11 右臂, 12~17 左手, 18~23 左臂）
    auto getForce = [this](int idx) -> QString
    {
        if (idx >= 0 && idx < static_cast<int>(force_sensor_.size()))
        {
            return QString::number(force_sensor_[idx], 'f', 3);
        }
        return "--";
    };

    rightHandFxLabel_->setText("Fx: " + getForce(0));
    rightHandFyLabel_->setText("Fy: " + getForce(1));
    rightHandFzLabel_->setText("Fz: " + getForce(2));
    rightHandMxLabel_->setText("Mx: " + getForce(3));
    rightHandMyLabel_->setText("My: " + getForce(4));
    rightHandMzLabel_->setText("Mz: " + getForce(5));

    rightArmFxLabel_->setText("Fx: " + getForce(6));
    rightArmFyLabel_->setText("Fy: " + getForce(7));
    rightArmFzLabel_->setText("Fz: " + getForce(8));
    rightArmMxLabel_->setText("Mx: " + getForce(9));
    rightArmMyLabel_->setText("My: " + getForce(10));
    rightArmMzLabel_->setText("Mz: " + getForce(11));

    leftHandFxLabel_->setText("Fx: " + getForce(12));
    leftHandFyLabel_->setText("Fy: " + getForce(13));
    leftHandFzLabel_->setText("Fz: " + getForce(14));
    leftHandMxLabel_->setText("Mx: " + getForce(15));
    leftHandMyLabel_->setText("My: " + getForce(16));
    leftHandMzLabel_->setText("Mz: " + getForce(17));

    leftArmFxLabel_->setText("Fx: " + getForce(18));
    leftArmFyLabel_->setText("Fy: " + getForce(19));
    leftArmFzLabel_->setText("Fz: " + getForce(20));
    leftArmMxLabel_->setText("Mx: " + getForce(21));
    leftArmMyLabel_->setText("My: " + getForce(22));
    leftArmMzLabel_->setText("Mz: " + getForce(23));
}

QString MainWindow::modeToString(int mode) const
{
    switch (mode)
    {
    case MODE_IDLE:
        return "安全停止 / 空闲";
    case MODE_LEFT_PRESET_POSITION:
        return "左臂恢复初始位置";
    case MODE_RIGHT_PRESET_POSITION:
        return "右臂恢复初始位置";
    case MODE_ACTIVE:
        return "左臂主动导纳";
    case MODE_RIGHT_ACTIVE:
        return "右臂主动导纳";
    case MODE_BILATERAL_ACTIVE:
        return "双侧主动导纳";
    case MODE_ZERO_FORCE:
        return "左臂零力拖动";
    case MODE_RIGHT_ZERO_FORCE:
        return "右臂零力拖动";
    case MODE_BILATERAL_ZERO_FORCE:
        return "双臂零力拖动";
    case MODE_PASSIVE:
        return "左臂被动正弦 (PID)";
    case MODE_PASSIVE_SINE_PD_FF:
        return "左臂被动正弦 (PD+FF)";
    default:
        return QString("未知(%1)").arg(mode);
    }
}

void MainWindow::appendLog(const QString &text)
{
    const QString now = QDateTime::currentDateTime().toString("HH:mm:ss");
    logTextEdit_->append(QString("[%1] %2").arg(now, text));
}

void MainWindow::updateIndicator(QLabel *indicator, QLabel *textLabel, bool ok, const QString &text)
{
    textLabel->setText(text);

    const char *status = ok ? "online" : "offline";
    if (indicator->property("status").toString() == status)
    {
        return;
    }

    indicator->setProperty("status", status);
    indicator->style()->unpolish(indicator);
    indicator->style()->polish(indicator);
}
