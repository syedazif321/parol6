/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *tab_control;
    QVBoxLayout *controlLayout;
    QGroupBox *groupBoxJointControl;
    QGridLayout *gridLayoutJointControl;
    QLabel *labelJoint1;
    QDoubleSpinBox *spinJoint1;
    QLabel *labelJoint2;
    QDoubleSpinBox *spinJoint2;
    QLabel *labelJoint3;
    QDoubleSpinBox *spinJoint3;
    QLabel *labelJoint4;
    QDoubleSpinBox *spinJoint4;
    QLabel *labelJoint5;
    QDoubleSpinBox *spinJoint5;
    QLabel *labelJoint6;
    QDoubleSpinBox *spinJoint6;
    QPushButton *btnApplyJointPositions;
    QGroupBox *groupBoxPoseControl;
    QGridLayout *gridLayoutPoseControl;
    QLabel *labelX;
    QDoubleSpinBox *spinX;
    QLabel *labelY;
    QDoubleSpinBox *spinY;
    QLabel *labelZ;
    QDoubleSpinBox *spinZ;
    QLabel *labelRoll;
    QDoubleSpinBox *spinRoll;
    QLabel *labelPitch;
    QDoubleSpinBox *spinPitch;
    QLabel *labelYaw;
    QDoubleSpinBox *spinYaw;
    QPushButton *btnMoveToPose;
    QGroupBox *groupBoxJogControls;
    QGridLayout *gridLayoutJog;
    QPushButton *btnJogXPlus;
    QPushButton *btnJogXMinus;
    QPushButton *btnJogYPlus;
    QPushButton *btnJogYMinus;
    QPushButton *btnJogZPlus;
    QPushButton *btnJogZMinus;
    QPushButton *btnJogRPlus;
    QPushButton *btnJogRMinus;
    QPushButton *btnJogPPlus;
    QPushButton *btnJogPMinus;
    QPushButton *btnJogYawPlus;
    QPushButton *btnJogYawMinus;
    QHBoxLayout *servoControls;
    QPushButton *btnServoOn;
    QPushButton *btnServoOff;
    QHBoxLayout *speedLayout;
    QLabel *labelSpeed;
    QSlider *sliderSpeed;
    QLabel *labelCurrentSpeed;
    QHBoxLayout *statusLayout;
    QLabel *labelBoxCount;
    QLabel *labelActiveSpeed;
    QLabel *labelErrors;
    QWidget *tab_targets;
    QVBoxLayout *targetLayout;
    QLineEdit *lineEditTargetName;
    QPushButton *btnSaveTarget;
    QHBoxLayout *targetDisplayButtons;
    QPushButton *btnShowJointValues;
    QPushButton *btnShowPoseValues;
    QHBoxLayout *targetDropdownLayout;
    QComboBox *comboBoxTargets;
    QPushButton *btnGoToTarget;
    QGridLayout *jointValuesLayout;
    QLabel *labelJoint1Value;
    QLabel *valueJoint1;
    QLabel *labelJoint2Value;
    QLabel *valueJoint2;
    QLabel *labelJoint3Value;
    QLabel *valueJoint3;
    QLabel *labelJoint4Value;
    QLabel *valueJoint4;
    QLabel *labelJoint5Value;
    QLabel *valueJoint5;
    QLabel *labelJoint6Value;
    QLabel *valueJoint6;
    QTextEdit *textTargetValues;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(507, 921);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab_control = new QWidget();
        tab_control->setObjectName(QString::fromUtf8("tab_control"));
        controlLayout = new QVBoxLayout(tab_control);
        controlLayout->setObjectName(QString::fromUtf8("controlLayout"));
        groupBoxJointControl = new QGroupBox(tab_control);
        groupBoxJointControl->setObjectName(QString::fromUtf8("groupBoxJointControl"));
        gridLayoutJointControl = new QGridLayout(groupBoxJointControl);
        gridLayoutJointControl->setObjectName(QString::fromUtf8("gridLayoutJointControl"));
        labelJoint1 = new QLabel(groupBoxJointControl);
        labelJoint1->setObjectName(QString::fromUtf8("labelJoint1"));

        gridLayoutJointControl->addWidget(labelJoint1, 0, 0, 1, 1);

        spinJoint1 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint1->setObjectName(QString::fromUtf8("spinJoint1"));

        gridLayoutJointControl->addWidget(spinJoint1, 0, 1, 1, 1);

        labelJoint2 = new QLabel(groupBoxJointControl);
        labelJoint2->setObjectName(QString::fromUtf8("labelJoint2"));

        gridLayoutJointControl->addWidget(labelJoint2, 1, 0, 1, 1);

        spinJoint2 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint2->setObjectName(QString::fromUtf8("spinJoint2"));

        gridLayoutJointControl->addWidget(spinJoint2, 1, 1, 1, 1);

        labelJoint3 = new QLabel(groupBoxJointControl);
        labelJoint3->setObjectName(QString::fromUtf8("labelJoint3"));

        gridLayoutJointControl->addWidget(labelJoint3, 2, 0, 1, 1);

        spinJoint3 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint3->setObjectName(QString::fromUtf8("spinJoint3"));

        gridLayoutJointControl->addWidget(spinJoint3, 2, 1, 1, 1);

        labelJoint4 = new QLabel(groupBoxJointControl);
        labelJoint4->setObjectName(QString::fromUtf8("labelJoint4"));

        gridLayoutJointControl->addWidget(labelJoint4, 3, 0, 1, 1);

        spinJoint4 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint4->setObjectName(QString::fromUtf8("spinJoint4"));

        gridLayoutJointControl->addWidget(spinJoint4, 3, 1, 1, 1);

        labelJoint5 = new QLabel(groupBoxJointControl);
        labelJoint5->setObjectName(QString::fromUtf8("labelJoint5"));

        gridLayoutJointControl->addWidget(labelJoint5, 4, 0, 1, 1);

        spinJoint5 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint5->setObjectName(QString::fromUtf8("spinJoint5"));

        gridLayoutJointControl->addWidget(spinJoint5, 4, 1, 1, 1);

        labelJoint6 = new QLabel(groupBoxJointControl);
        labelJoint6->setObjectName(QString::fromUtf8("labelJoint6"));

        gridLayoutJointControl->addWidget(labelJoint6, 5, 0, 1, 1);

        spinJoint6 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint6->setObjectName(QString::fromUtf8("spinJoint6"));

        gridLayoutJointControl->addWidget(spinJoint6, 5, 1, 1, 1);

        btnApplyJointPositions = new QPushButton(groupBoxJointControl);
        btnApplyJointPositions->setObjectName(QString::fromUtf8("btnApplyJointPositions"));

        gridLayoutJointControl->addWidget(btnApplyJointPositions, 6, 0, 1, 2);


        controlLayout->addWidget(groupBoxJointControl);

        groupBoxPoseControl = new QGroupBox(tab_control);
        groupBoxPoseControl->setObjectName(QString::fromUtf8("groupBoxPoseControl"));
        gridLayoutPoseControl = new QGridLayout(groupBoxPoseControl);
        gridLayoutPoseControl->setObjectName(QString::fromUtf8("gridLayoutPoseControl"));
        labelX = new QLabel(groupBoxPoseControl);
        labelX->setObjectName(QString::fromUtf8("labelX"));

        gridLayoutPoseControl->addWidget(labelX, 0, 0, 1, 1);

        spinX = new QDoubleSpinBox(groupBoxPoseControl);
        spinX->setObjectName(QString::fromUtf8("spinX"));

        gridLayoutPoseControl->addWidget(spinX, 0, 1, 1, 1);

        labelY = new QLabel(groupBoxPoseControl);
        labelY->setObjectName(QString::fromUtf8("labelY"));

        gridLayoutPoseControl->addWidget(labelY, 1, 0, 1, 1);

        spinY = new QDoubleSpinBox(groupBoxPoseControl);
        spinY->setObjectName(QString::fromUtf8("spinY"));

        gridLayoutPoseControl->addWidget(spinY, 1, 1, 1, 1);

        labelZ = new QLabel(groupBoxPoseControl);
        labelZ->setObjectName(QString::fromUtf8("labelZ"));

        gridLayoutPoseControl->addWidget(labelZ, 2, 0, 1, 1);

        spinZ = new QDoubleSpinBox(groupBoxPoseControl);
        spinZ->setObjectName(QString::fromUtf8("spinZ"));

        gridLayoutPoseControl->addWidget(spinZ, 2, 1, 1, 1);

        labelRoll = new QLabel(groupBoxPoseControl);
        labelRoll->setObjectName(QString::fromUtf8("labelRoll"));

        gridLayoutPoseControl->addWidget(labelRoll, 3, 0, 1, 1);

        spinRoll = new QDoubleSpinBox(groupBoxPoseControl);
        spinRoll->setObjectName(QString::fromUtf8("spinRoll"));

        gridLayoutPoseControl->addWidget(spinRoll, 3, 1, 1, 1);

        labelPitch = new QLabel(groupBoxPoseControl);
        labelPitch->setObjectName(QString::fromUtf8("labelPitch"));

        gridLayoutPoseControl->addWidget(labelPitch, 4, 0, 1, 1);

        spinPitch = new QDoubleSpinBox(groupBoxPoseControl);
        spinPitch->setObjectName(QString::fromUtf8("spinPitch"));

        gridLayoutPoseControl->addWidget(spinPitch, 4, 1, 1, 1);

        labelYaw = new QLabel(groupBoxPoseControl);
        labelYaw->setObjectName(QString::fromUtf8("labelYaw"));

        gridLayoutPoseControl->addWidget(labelYaw, 5, 0, 1, 1);

        spinYaw = new QDoubleSpinBox(groupBoxPoseControl);
        spinYaw->setObjectName(QString::fromUtf8("spinYaw"));

        gridLayoutPoseControl->addWidget(spinYaw, 5, 1, 1, 1);

        btnMoveToPose = new QPushButton(groupBoxPoseControl);
        btnMoveToPose->setObjectName(QString::fromUtf8("btnMoveToPose"));

        gridLayoutPoseControl->addWidget(btnMoveToPose, 6, 0, 1, 2);


        controlLayout->addWidget(groupBoxPoseControl);

        groupBoxJogControls = new QGroupBox(tab_control);
        groupBoxJogControls->setObjectName(QString::fromUtf8("groupBoxJogControls"));
        gridLayoutJog = new QGridLayout(groupBoxJogControls);
        gridLayoutJog->setObjectName(QString::fromUtf8("gridLayoutJog"));
        btnJogXPlus = new QPushButton(groupBoxJogControls);
        btnJogXPlus->setObjectName(QString::fromUtf8("btnJogXPlus"));

        gridLayoutJog->addWidget(btnJogXPlus, 0, 0, 1, 1);

        btnJogXMinus = new QPushButton(groupBoxJogControls);
        btnJogXMinus->setObjectName(QString::fromUtf8("btnJogXMinus"));

        gridLayoutJog->addWidget(btnJogXMinus, 0, 1, 1, 1);

        btnJogYPlus = new QPushButton(groupBoxJogControls);
        btnJogYPlus->setObjectName(QString::fromUtf8("btnJogYPlus"));

        gridLayoutJog->addWidget(btnJogYPlus, 1, 0, 1, 1);

        btnJogYMinus = new QPushButton(groupBoxJogControls);
        btnJogYMinus->setObjectName(QString::fromUtf8("btnJogYMinus"));

        gridLayoutJog->addWidget(btnJogYMinus, 1, 1, 1, 1);

        btnJogZPlus = new QPushButton(groupBoxJogControls);
        btnJogZPlus->setObjectName(QString::fromUtf8("btnJogZPlus"));

        gridLayoutJog->addWidget(btnJogZPlus, 2, 0, 1, 1);

        btnJogZMinus = new QPushButton(groupBoxJogControls);
        btnJogZMinus->setObjectName(QString::fromUtf8("btnJogZMinus"));

        gridLayoutJog->addWidget(btnJogZMinus, 2, 1, 1, 1);

        btnJogRPlus = new QPushButton(groupBoxJogControls);
        btnJogRPlus->setObjectName(QString::fromUtf8("btnJogRPlus"));

        gridLayoutJog->addWidget(btnJogRPlus, 3, 0, 1, 1);

        btnJogRMinus = new QPushButton(groupBoxJogControls);
        btnJogRMinus->setObjectName(QString::fromUtf8("btnJogRMinus"));

        gridLayoutJog->addWidget(btnJogRMinus, 3, 1, 1, 1);

        btnJogPPlus = new QPushButton(groupBoxJogControls);
        btnJogPPlus->setObjectName(QString::fromUtf8("btnJogPPlus"));

        gridLayoutJog->addWidget(btnJogPPlus, 4, 0, 1, 1);

        btnJogPMinus = new QPushButton(groupBoxJogControls);
        btnJogPMinus->setObjectName(QString::fromUtf8("btnJogPMinus"));

        gridLayoutJog->addWidget(btnJogPMinus, 4, 1, 1, 1);

        btnJogYawPlus = new QPushButton(groupBoxJogControls);
        btnJogYawPlus->setObjectName(QString::fromUtf8("btnJogYawPlus"));

        gridLayoutJog->addWidget(btnJogYawPlus, 5, 0, 1, 1);

        btnJogYawMinus = new QPushButton(groupBoxJogControls);
        btnJogYawMinus->setObjectName(QString::fromUtf8("btnJogYawMinus"));

        gridLayoutJog->addWidget(btnJogYawMinus, 5, 1, 1, 1);


        controlLayout->addWidget(groupBoxJogControls);

        speedLayout = new QHBoxLayout();

        servoControls = new QHBoxLayout();
        servoControls->setObjectName(QString::fromUtf8("servoControls"));
        btnServoOn = new QPushButton(tab_control);
        btnServoOn->setObjectName(QString::fromUtf8("btnServoOn"));

        servoControls->addWidget(btnServoOn);

        btnServoOff = new QPushButton(tab_control);
        btnServoOff->setObjectName(QString::fromUtf8("btnServoOff"));

        servoControls->addWidget(btnServoOff);


        controlLayout->addLayout(servoControls);
        speedLayout->setObjectName(QString::fromUtf8("speedLayout"));
        labelSpeed = new QLabel(tab_control);
        labelSpeed->setObjectName(QString::fromUtf8("labelSpeed"));

        speedLayout->addWidget(labelSpeed);

        sliderSpeed = new QSlider(tab_control);
        sliderSpeed->setObjectName(QString::fromUtf8("sliderSpeed"));
        sliderSpeed->setMinimum(50);
        sliderSpeed->setMaximum(200);
        sliderSpeed->setValue(100);
        sliderSpeed->setOrientation(Qt::Horizontal);

        speedLayout->addWidget(sliderSpeed);

        labelCurrentSpeed = new QLabel(tab_control);
        labelCurrentSpeed->setObjectName(QString::fromUtf8("labelCurrentSpeed"));

        speedLayout->addWidget(labelCurrentSpeed);


        controlLayout->addLayout(speedLayout);

        statusLayout = new QHBoxLayout();
        statusLayout->setObjectName(QString::fromUtf8("statusLayout"));
        labelBoxCount = new QLabel(tab_control);
        labelBoxCount->setObjectName(QString::fromUtf8("labelBoxCount"));

        statusLayout->addWidget(labelBoxCount);

        labelActiveSpeed = new QLabel(tab_control);
        labelActiveSpeed->setObjectName(QString::fromUtf8("labelActiveSpeed"));

        statusLayout->addWidget(labelActiveSpeed);

        labelErrors = new QLabel(tab_control);
        labelErrors->setObjectName(QString::fromUtf8("labelErrors"));

        statusLayout->addWidget(labelErrors);


        controlLayout->addLayout(statusLayout);

        tabWidget->addTab(tab_control, QString());
        tab_targets = new QWidget();
        tab_targets->setObjectName(QString::fromUtf8("tab_targets"));
        targetLayout = new QVBoxLayout(tab_targets);
        targetLayout->setObjectName(QString::fromUtf8("targetLayout"));
        lineEditTargetName = new QLineEdit(tab_targets);
        lineEditTargetName->setObjectName(QString::fromUtf8("lineEditTargetName"));

        targetLayout->addWidget(lineEditTargetName);

        btnSaveTarget = new QPushButton(tab_targets);
        btnSaveTarget->setObjectName(QString::fromUtf8("btnSaveTarget"));

        targetLayout->addWidget(btnSaveTarget);

        targetDisplayButtons = new QHBoxLayout();
        targetDisplayButtons->setObjectName(QString::fromUtf8("targetDisplayButtons"));
        btnShowJointValues = new QPushButton(tab_targets);
        btnShowJointValues->setObjectName(QString::fromUtf8("btnShowJointValues"));

        targetDisplayButtons->addWidget(btnShowJointValues);

        btnShowPoseValues = new QPushButton(tab_targets);
        btnShowPoseValues->setObjectName(QString::fromUtf8("btnShowPoseValues"));

        targetDisplayButtons->addWidget(btnShowPoseValues);


        targetLayout->addLayout(targetDisplayButtons);

        targetDropdownLayout = new QHBoxLayout();
        targetDropdownLayout->setObjectName(QString::fromUtf8("targetDropdownLayout"));
        comboBoxTargets = new QComboBox(tab_targets);
        comboBoxTargets->setObjectName(QString::fromUtf8("comboBoxTargets"));

        targetDropdownLayout->addWidget(comboBoxTargets);

        btnGoToTarget = new QPushButton(tab_targets);
        btnGoToTarget->setObjectName(QString::fromUtf8("btnGoToTarget"));

        targetDropdownLayout->addWidget(btnGoToTarget);


        targetLayout->addLayout(targetDropdownLayout);

        jointValuesLayout = new QGridLayout();
        jointValuesLayout->setObjectName(QString::fromUtf8("jointValuesLayout"));
        labelJoint1Value = new QLabel(tab_targets);
        labelJoint1Value->setObjectName(QString::fromUtf8("labelJoint1Value"));

        jointValuesLayout->addWidget(labelJoint1Value, 0, 0, 1, 1);

        valueJoint1 = new QLabel(tab_targets);
        valueJoint1->setObjectName(QString::fromUtf8("valueJoint1"));

        jointValuesLayout->addWidget(valueJoint1, 0, 1, 1, 1);

        labelJoint2Value = new QLabel(tab_targets);
        labelJoint2Value->setObjectName(QString::fromUtf8("labelJoint2Value"));

        jointValuesLayout->addWidget(labelJoint2Value, 1, 0, 1, 1);

        valueJoint2 = new QLabel(tab_targets);
        valueJoint2->setObjectName(QString::fromUtf8("valueJoint2"));

        jointValuesLayout->addWidget(valueJoint2, 1, 1, 1, 1);

        labelJoint3Value = new QLabel(tab_targets);
        labelJoint3Value->setObjectName(QString::fromUtf8("labelJoint3Value"));

        jointValuesLayout->addWidget(labelJoint3Value, 2, 0, 1, 1);

        valueJoint3 = new QLabel(tab_targets);
        valueJoint3->setObjectName(QString::fromUtf8("valueJoint3"));

        jointValuesLayout->addWidget(valueJoint3, 2, 1, 1, 1);

        labelJoint4Value = new QLabel(tab_targets);
        labelJoint4Value->setObjectName(QString::fromUtf8("labelJoint4Value"));

        jointValuesLayout->addWidget(labelJoint4Value, 3, 0, 1, 1);

        valueJoint4 = new QLabel(tab_targets);
        valueJoint4->setObjectName(QString::fromUtf8("valueJoint4"));

        jointValuesLayout->addWidget(valueJoint4, 3, 1, 1, 1);

        labelJoint5Value = new QLabel(tab_targets);
        labelJoint5Value->setObjectName(QString::fromUtf8("labelJoint5Value"));

        jointValuesLayout->addWidget(labelJoint5Value, 4, 0, 1, 1);

        valueJoint5 = new QLabel(tab_targets);
        valueJoint5->setObjectName(QString::fromUtf8("valueJoint5"));

        jointValuesLayout->addWidget(valueJoint5, 4, 1, 1, 1);

        labelJoint6Value = new QLabel(tab_targets);
        labelJoint6Value->setObjectName(QString::fromUtf8("labelJoint6Value"));

        jointValuesLayout->addWidget(labelJoint6Value, 5, 0, 1, 1);

        valueJoint6 = new QLabel(tab_targets);
        valueJoint6->setObjectName(QString::fromUtf8("valueJoint6"));

        jointValuesLayout->addWidget(valueJoint6, 5, 1, 1, 1);


        targetLayout->addLayout(jointValuesLayout);

        textTargetValues = new QTextEdit(tab_targets);
        textTargetValues->setObjectName(QString::fromUtf8("textTargetValues"));
        textTargetValues->setReadOnly(true);

        targetLayout->addWidget(textTargetValues);

        tabWidget->addTab(tab_targets, QString());

        verticalLayout->addWidget(tabWidget);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 507, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Parol6 Robotic Arm Control", nullptr));
        groupBoxJointControl->setTitle(QCoreApplication::translate("MainWindow", "Joint Control", nullptr));
        labelJoint1->setText(QCoreApplication::translate("MainWindow", "Joint 1:", nullptr));
        labelJoint2->setText(QCoreApplication::translate("MainWindow", "Joint 2:", nullptr));
        labelJoint3->setText(QCoreApplication::translate("MainWindow", "Joint 3:", nullptr));
        labelJoint4->setText(QCoreApplication::translate("MainWindow", "Joint 4:", nullptr));
        labelJoint5->setText(QCoreApplication::translate("MainWindow", "Joint 5:", nullptr));
        labelJoint6->setText(QCoreApplication::translate("MainWindow", "Joint 6:", nullptr));
        btnApplyJointPositions->setText(QCoreApplication::translate("MainWindow", "Apply Joint Positions", nullptr));
        groupBoxPoseControl->setTitle(QCoreApplication::translate("MainWindow", "Cartesian Pose Control", nullptr));
        labelX->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        labelY->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        labelZ->setText(QCoreApplication::translate("MainWindow", "Z:", nullptr));
        labelRoll->setText(QCoreApplication::translate("MainWindow", "Roll:", nullptr));
        labelPitch->setText(QCoreApplication::translate("MainWindow", "Pitch:", nullptr));
        labelYaw->setText(QCoreApplication::translate("MainWindow", "Yaw:", nullptr));
        btnMoveToPose->setText(QCoreApplication::translate("MainWindow", "Move End-Effector", nullptr));
        groupBoxJogControls->setTitle(QCoreApplication::translate("MainWindow", "Jog Controls", nullptr));
        btnJogXPlus->setText(QCoreApplication::translate("MainWindow", "X+", nullptr));
        btnJogXMinus->setText(QCoreApplication::translate("MainWindow", "X-", nullptr));
        btnJogYPlus->setText(QCoreApplication::translate("MainWindow", "Y+", nullptr));
        btnJogYMinus->setText(QCoreApplication::translate("MainWindow", "Y-", nullptr));
        btnJogZPlus->setText(QCoreApplication::translate("MainWindow", "Z+", nullptr));
        btnJogZMinus->setText(QCoreApplication::translate("MainWindow", "Z-", nullptr));
        btnJogRPlus->setText(QCoreApplication::translate("MainWindow", "Roll+", nullptr));
        btnJogRMinus->setText(QCoreApplication::translate("MainWindow", "Roll-", nullptr));
        btnJogPPlus->setText(QCoreApplication::translate("MainWindow", "Pitch+", nullptr));
        btnJogPMinus->setText(QCoreApplication::translate("MainWindow", "Pitch-", nullptr));
        btnJogYawPlus->setText(QCoreApplication::translate("MainWindow", "Yaw+", nullptr));
        btnJogYawMinus->setText(QCoreApplication::translate("MainWindow", "Yaw-", nullptr));
        btnServoOn->setText(QCoreApplication::translate("MainWindow", "Servo On", nullptr));
        btnServoOff->setText(QCoreApplication::translate("MainWindow", "Servo Off", nullptr));
        labelSpeed->setText(QCoreApplication::translate("MainWindow", "Speed (0.5x - 2x):", nullptr));
        labelCurrentSpeed->setText(QCoreApplication::translate("MainWindow", "1.0x", nullptr));
        labelBoxCount->setText(QCoreApplication::translate("MainWindow", "Boxes: 0", nullptr));
        labelActiveSpeed->setText(QCoreApplication::translate("MainWindow", "Speed: 1.0x", nullptr));
        labelErrors->setText(QCoreApplication::translate("MainWindow", "Errors: None", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_control), QCoreApplication::translate("MainWindow", "Control", nullptr));
        lineEditTargetName->setPlaceholderText(QCoreApplication::translate("MainWindow", "Enter target name...", nullptr));
        btnSaveTarget->setText(QCoreApplication::translate("MainWindow", "Save Target", nullptr));
        btnShowJointValues->setText(QCoreApplication::translate("MainWindow", "Show Joint Values", nullptr));
        btnShowPoseValues->setText(QCoreApplication::translate("MainWindow", "Show Pose Values", nullptr));
        btnGoToTarget->setText(QCoreApplication::translate("MainWindow", "Go to Target", nullptr));
        labelJoint1Value->setText(QCoreApplication::translate("MainWindow", "Joint1:", nullptr));
        valueJoint1->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint2Value->setText(QCoreApplication::translate("MainWindow", "Joint2:", nullptr));
        valueJoint2->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint3Value->setText(QCoreApplication::translate("MainWindow", "Joint3:", nullptr));
        valueJoint3->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint4Value->setText(QCoreApplication::translate("MainWindow", "Joint4:", nullptr));
        valueJoint4->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint5Value->setText(QCoreApplication::translate("MainWindow", "Joint5:", nullptr));
        valueJoint5->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint6Value->setText(QCoreApplication::translate("MainWindow", "Joint6:", nullptr));
        valueJoint6->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_targets), QCoreApplication::translate("MainWindow", "Targets", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_H
