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
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
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
    QVBoxLayout *verticalLayout_4;
    QGroupBox *groupBoxJointControl;
    QGridLayout *gridLayoutJointControl;
    QDoubleSpinBox *spinJoint4;
    QDoubleSpinBox *spinJoint3;
    QLabel *labelJoint5;
    QDoubleSpinBox *spinJoint1;
    QLabel *labelJoint4;
    QLabel *labelJoint6;
    QLabel *labelJoint1;
    QDoubleSpinBox *spinJoint2;
    QDoubleSpinBox *spinJoint5;
    QDoubleSpinBox *spinJoint6;
    QLabel *labelJoint3;
    QLabel *labelJoint2;
    QPushButton *btnApplyJointPositions;
    QGroupBox *groupBoxPoseControl;
    QGridLayout *gridLayoutPoseControl;
    QDoubleSpinBox *spinRoll;
    QLabel *labelRoll;
    QDoubleSpinBox *spinY;
    QDoubleSpinBox *spinX;
    QPushButton *btnMoveToPose;
    QDoubleSpinBox *spinZ;
    QLabel *labelZ;
    QLabel *labelYaw;
    QDoubleSpinBox *spinYaw;
    QLabel *labelX;
    QLabel *labelY;
    QLabel *labelPitch;
    QDoubleSpinBox *spinPitch;
    QGroupBox *groupBoxJogControls;
    QGridLayout *gridLayoutJog;
    QPushButton *btnJogYPlus;
    QPushButton *btnJogZPlus;
    QPushButton *btnJogPPlus;
    QPushButton *btnJogXMinus;
    QPushButton *btnJogZMinus;
    QPushButton *btnServoOff;
    QPushButton *btnJogPMinus;
    QPushButton *btnJogRMinus;
    QPushButton *btnJogXPlus;
    QPushButton *btnJogYawPlus;
    QPushButton *btnJogRPlus;
    QPushButton *btnServoOn;
    QPushButton *btnJogYMinus;
    QPushButton *btnJogYawMinus;
    QHBoxLayout *speedLayout;
    QLabel *labelSpeed;
    QSlider *sliderSpeed;
    QLabel *labelCurrentSpeed;
    QHBoxLayout *statusLayout;
    QLabel *labelBoxCount;
    QLabel *labelActiveSpeed;
    QLabel *labelErrors;
    QWidget *tab_targets;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QPushButton *btnShowJointValues;
    QPushButton *btnShowPoseValues;
    QHBoxLayout *targetDropdownLayout;
    QLineEdit *lineEditTargetName;
    QComboBox *comboBoxTargets;
    QPushButton *btnGoToTarget;
    QPushButton *btnSaveTarget;
    QPushButton *btnSaveTarget_2;
    QGridLayout *jointValuesLayout;
    QLabel *labelJoint2Value;
    QLabel *labelJoint3Value;
    QLabel *labelJoint4Value;
    QLabel *labelJoint5Value;
    QLabel *labelJoint1Value;
    QLabel *labelJoint6Value;
    QLabel *valueJoint1;
    QLabel *valueJoint3;
    QLabel *valueJoint5;
    QLabel *valueJoint2;
    QLabel *valueJoint4;
    QLabel *valueJoint6;
    QSpacerItem *horizontalSpacer;
    QFrame *frame;
    QGridLayout *jointValuesLayout_3;
    QLabel *valueJoint6_3;
    QLabel *labelJoint3Value_3;
    QLabel *labelJoint5Value_3;
    QLabel *labelJoint2Value_3;
    QLabel *valueJoint1_3;
    QLabel *valueJoint5_3;
    QLabel *valueJoint3_3;
    QLabel *labelJoint4Value_3;
    QLabel *valueJoint4_3;
    QLabel *labelJoint6Value_3;
    QLabel *labelJoint1Value_3;
    QLabel *valueJoint2_3;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *btnStartProgram;
    QPushButton *btnSopProgram;
    QTextEdit *textTargetValues;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(436, 952);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab_control = new QWidget();
        tab_control->setObjectName(QString::fromUtf8("tab_control"));
        verticalLayout_4 = new QVBoxLayout(tab_control);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        groupBoxJointControl = new QGroupBox(tab_control);
        groupBoxJointControl->setObjectName(QString::fromUtf8("groupBoxJointControl"));
        gridLayoutJointControl = new QGridLayout(groupBoxJointControl);
        gridLayoutJointControl->setObjectName(QString::fromUtf8("gridLayoutJointControl"));
        spinJoint4 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint4->setObjectName(QString::fromUtf8("spinJoint4"));

        gridLayoutJointControl->addWidget(spinJoint4, 3, 1, 1, 1);

        spinJoint3 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint3->setObjectName(QString::fromUtf8("spinJoint3"));

        gridLayoutJointControl->addWidget(spinJoint3, 2, 1, 1, 1);

        labelJoint5 = new QLabel(groupBoxJointControl);
        labelJoint5->setObjectName(QString::fromUtf8("labelJoint5"));

        gridLayoutJointControl->addWidget(labelJoint5, 4, 0, 1, 1);

        spinJoint1 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint1->setObjectName(QString::fromUtf8("spinJoint1"));

        gridLayoutJointControl->addWidget(spinJoint1, 0, 1, 1, 1);

        labelJoint4 = new QLabel(groupBoxJointControl);
        labelJoint4->setObjectName(QString::fromUtf8("labelJoint4"));

        gridLayoutJointControl->addWidget(labelJoint4, 3, 0, 1, 1);

        labelJoint6 = new QLabel(groupBoxJointControl);
        labelJoint6->setObjectName(QString::fromUtf8("labelJoint6"));

        gridLayoutJointControl->addWidget(labelJoint6, 5, 0, 1, 1);

        labelJoint1 = new QLabel(groupBoxJointControl);
        labelJoint1->setObjectName(QString::fromUtf8("labelJoint1"));

        gridLayoutJointControl->addWidget(labelJoint1, 0, 0, 1, 1);

        spinJoint2 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint2->setObjectName(QString::fromUtf8("spinJoint2"));

        gridLayoutJointControl->addWidget(spinJoint2, 1, 1, 1, 1);

        spinJoint5 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint5->setObjectName(QString::fromUtf8("spinJoint5"));

        gridLayoutJointControl->addWidget(spinJoint5, 4, 1, 1, 1);

        spinJoint6 = new QDoubleSpinBox(groupBoxJointControl);
        spinJoint6->setObjectName(QString::fromUtf8("spinJoint6"));

        gridLayoutJointControl->addWidget(spinJoint6, 5, 1, 1, 1);

        labelJoint3 = new QLabel(groupBoxJointControl);
        labelJoint3->setObjectName(QString::fromUtf8("labelJoint3"));

        gridLayoutJointControl->addWidget(labelJoint3, 2, 0, 1, 1);

        labelJoint2 = new QLabel(groupBoxJointControl);
        labelJoint2->setObjectName(QString::fromUtf8("labelJoint2"));

        gridLayoutJointControl->addWidget(labelJoint2, 1, 0, 1, 1);

        btnApplyJointPositions = new QPushButton(groupBoxJointControl);
        btnApplyJointPositions->setObjectName(QString::fromUtf8("btnApplyJointPositions"));

        gridLayoutJointControl->addWidget(btnApplyJointPositions, 6, 0, 1, 2);


        verticalLayout_4->addWidget(groupBoxJointControl);

        groupBoxPoseControl = new QGroupBox(tab_control);
        groupBoxPoseControl->setObjectName(QString::fromUtf8("groupBoxPoseControl"));
        gridLayoutPoseControl = new QGridLayout(groupBoxPoseControl);
        gridLayoutPoseControl->setObjectName(QString::fromUtf8("gridLayoutPoseControl"));
        spinRoll = new QDoubleSpinBox(groupBoxPoseControl);
        spinRoll->setObjectName(QString::fromUtf8("spinRoll"));

        gridLayoutPoseControl->addWidget(spinRoll, 3, 1, 1, 1);

        labelRoll = new QLabel(groupBoxPoseControl);
        labelRoll->setObjectName(QString::fromUtf8("labelRoll"));

        gridLayoutPoseControl->addWidget(labelRoll, 3, 0, 1, 1);

        spinY = new QDoubleSpinBox(groupBoxPoseControl);
        spinY->setObjectName(QString::fromUtf8("spinY"));

        gridLayoutPoseControl->addWidget(spinY, 1, 1, 1, 1);

        spinX = new QDoubleSpinBox(groupBoxPoseControl);
        spinX->setObjectName(QString::fromUtf8("spinX"));

        gridLayoutPoseControl->addWidget(spinX, 0, 1, 1, 1);

        btnMoveToPose = new QPushButton(groupBoxPoseControl);
        btnMoveToPose->setObjectName(QString::fromUtf8("btnMoveToPose"));

        gridLayoutPoseControl->addWidget(btnMoveToPose, 6, 0, 1, 2);

        spinZ = new QDoubleSpinBox(groupBoxPoseControl);
        spinZ->setObjectName(QString::fromUtf8("spinZ"));

        gridLayoutPoseControl->addWidget(spinZ, 2, 1, 1, 1);

        labelZ = new QLabel(groupBoxPoseControl);
        labelZ->setObjectName(QString::fromUtf8("labelZ"));

        gridLayoutPoseControl->addWidget(labelZ, 2, 0, 1, 1);

        labelYaw = new QLabel(groupBoxPoseControl);
        labelYaw->setObjectName(QString::fromUtf8("labelYaw"));

        gridLayoutPoseControl->addWidget(labelYaw, 5, 0, 1, 1);

        spinYaw = new QDoubleSpinBox(groupBoxPoseControl);
        spinYaw->setObjectName(QString::fromUtf8("spinYaw"));

        gridLayoutPoseControl->addWidget(spinYaw, 5, 1, 1, 1);

        labelX = new QLabel(groupBoxPoseControl);
        labelX->setObjectName(QString::fromUtf8("labelX"));

        gridLayoutPoseControl->addWidget(labelX, 0, 0, 1, 1);

        labelY = new QLabel(groupBoxPoseControl);
        labelY->setObjectName(QString::fromUtf8("labelY"));

        gridLayoutPoseControl->addWidget(labelY, 1, 0, 1, 1);

        labelPitch = new QLabel(groupBoxPoseControl);
        labelPitch->setObjectName(QString::fromUtf8("labelPitch"));

        gridLayoutPoseControl->addWidget(labelPitch, 4, 0, 1, 1);

        spinPitch = new QDoubleSpinBox(groupBoxPoseControl);
        spinPitch->setObjectName(QString::fromUtf8("spinPitch"));

        gridLayoutPoseControl->addWidget(spinPitch, 4, 1, 1, 1);


        verticalLayout_4->addWidget(groupBoxPoseControl);

        groupBoxJogControls = new QGroupBox(tab_control);
        groupBoxJogControls->setObjectName(QString::fromUtf8("groupBoxJogControls"));
        gridLayoutJog = new QGridLayout(groupBoxJogControls);
        gridLayoutJog->setObjectName(QString::fromUtf8("gridLayoutJog"));
        btnJogYPlus = new QPushButton(groupBoxJogControls);
        btnJogYPlus->setObjectName(QString::fromUtf8("btnJogYPlus"));

        gridLayoutJog->addWidget(btnJogYPlus, 1, 0, 1, 1);

        btnJogZPlus = new QPushButton(groupBoxJogControls);
        btnJogZPlus->setObjectName(QString::fromUtf8("btnJogZPlus"));

        gridLayoutJog->addWidget(btnJogZPlus, 2, 0, 1, 1);

        btnJogPPlus = new QPushButton(groupBoxJogControls);
        btnJogPPlus->setObjectName(QString::fromUtf8("btnJogPPlus"));

        gridLayoutJog->addWidget(btnJogPPlus, 4, 0, 1, 1);

        btnJogXMinus = new QPushButton(groupBoxJogControls);
        btnJogXMinus->setObjectName(QString::fromUtf8("btnJogXMinus"));

        gridLayoutJog->addWidget(btnJogXMinus, 0, 1, 1, 1);

        btnJogZMinus = new QPushButton(groupBoxJogControls);
        btnJogZMinus->setObjectName(QString::fromUtf8("btnJogZMinus"));

        gridLayoutJog->addWidget(btnJogZMinus, 2, 1, 1, 1);

        btnServoOff = new QPushButton(groupBoxJogControls);
        btnServoOff->setObjectName(QString::fromUtf8("btnServoOff"));

        gridLayoutJog->addWidget(btnServoOff, 6, 1, 1, 1);

        btnJogPMinus = new QPushButton(groupBoxJogControls);
        btnJogPMinus->setObjectName(QString::fromUtf8("btnJogPMinus"));

        gridLayoutJog->addWidget(btnJogPMinus, 4, 1, 1, 1);

        btnJogRMinus = new QPushButton(groupBoxJogControls);
        btnJogRMinus->setObjectName(QString::fromUtf8("btnJogRMinus"));

        gridLayoutJog->addWidget(btnJogRMinus, 3, 1, 1, 1);

        btnJogXPlus = new QPushButton(groupBoxJogControls);
        btnJogXPlus->setObjectName(QString::fromUtf8("btnJogXPlus"));

        gridLayoutJog->addWidget(btnJogXPlus, 0, 0, 1, 1);

        btnJogYawPlus = new QPushButton(groupBoxJogControls);
        btnJogYawPlus->setObjectName(QString::fromUtf8("btnJogYawPlus"));

        gridLayoutJog->addWidget(btnJogYawPlus, 5, 0, 1, 1);

        btnJogRPlus = new QPushButton(groupBoxJogControls);
        btnJogRPlus->setObjectName(QString::fromUtf8("btnJogRPlus"));

        gridLayoutJog->addWidget(btnJogRPlus, 3, 0, 1, 1);

        btnServoOn = new QPushButton(groupBoxJogControls);
        btnServoOn->setObjectName(QString::fromUtf8("btnServoOn"));

        gridLayoutJog->addWidget(btnServoOn, 6, 0, 1, 1);

        btnJogYMinus = new QPushButton(groupBoxJogControls);
        btnJogYMinus->setObjectName(QString::fromUtf8("btnJogYMinus"));

        gridLayoutJog->addWidget(btnJogYMinus, 1, 1, 1, 1);

        btnJogYawMinus = new QPushButton(groupBoxJogControls);
        btnJogYawMinus->setObjectName(QString::fromUtf8("btnJogYawMinus"));

        gridLayoutJog->addWidget(btnJogYawMinus, 5, 1, 1, 1);


        verticalLayout_4->addWidget(groupBoxJogControls);

        speedLayout = new QHBoxLayout();
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


        verticalLayout_4->addLayout(speedLayout);

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


        verticalLayout_4->addLayout(statusLayout);

        tabWidget->addTab(tab_control, QString());
        tab_targets = new QWidget();
        tab_targets->setObjectName(QString::fromUtf8("tab_targets"));
        verticalLayout_3 = new QVBoxLayout(tab_targets);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));

        verticalLayout_3->addLayout(verticalLayout_2);

        btnShowJointValues = new QPushButton(tab_targets);
        btnShowJointValues->setObjectName(QString::fromUtf8("btnShowJointValues"));

        verticalLayout_3->addWidget(btnShowJointValues);

        btnShowPoseValues = new QPushButton(tab_targets);
        btnShowPoseValues->setObjectName(QString::fromUtf8("btnShowPoseValues"));

        verticalLayout_3->addWidget(btnShowPoseValues);

        targetDropdownLayout = new QHBoxLayout();
        targetDropdownLayout->setObjectName(QString::fromUtf8("targetDropdownLayout"));
        lineEditTargetName = new QLineEdit(tab_targets);
        lineEditTargetName->setObjectName(QString::fromUtf8("lineEditTargetName"));

        targetDropdownLayout->addWidget(lineEditTargetName);

        comboBoxTargets = new QComboBox(tab_targets);
        comboBoxTargets->setObjectName(QString::fromUtf8("comboBoxTargets"));

        targetDropdownLayout->addWidget(comboBoxTargets);

        btnGoToTarget = new QPushButton(tab_targets);
        btnGoToTarget->setObjectName(QString::fromUtf8("btnGoToTarget"));

        targetDropdownLayout->addWidget(btnGoToTarget);


        verticalLayout_3->addLayout(targetDropdownLayout);

        btnSaveTarget = new QPushButton(tab_targets);
        btnSaveTarget->setObjectName(QString::fromUtf8("btnSaveTarget"));

        verticalLayout_3->addWidget(btnSaveTarget);

        btnSaveTarget_2 = new QPushButton(tab_targets);
        btnSaveTarget_2->setObjectName(QString::fromUtf8("btnSaveTarget_2"));

        verticalLayout_3->addWidget(btnSaveTarget_2);

        jointValuesLayout = new QGridLayout();
        jointValuesLayout->setObjectName(QString::fromUtf8("jointValuesLayout"));
        labelJoint2Value = new QLabel(tab_targets);
        labelJoint2Value->setObjectName(QString::fromUtf8("labelJoint2Value"));

        jointValuesLayout->addWidget(labelJoint2Value, 3, 0, 1, 1);

        labelJoint3Value = new QLabel(tab_targets);
        labelJoint3Value->setObjectName(QString::fromUtf8("labelJoint3Value"));

        jointValuesLayout->addWidget(labelJoint3Value, 4, 0, 1, 1);

        labelJoint4Value = new QLabel(tab_targets);
        labelJoint4Value->setObjectName(QString::fromUtf8("labelJoint4Value"));

        jointValuesLayout->addWidget(labelJoint4Value, 5, 0, 1, 1);

        labelJoint5Value = new QLabel(tab_targets);
        labelJoint5Value->setObjectName(QString::fromUtf8("labelJoint5Value"));

        jointValuesLayout->addWidget(labelJoint5Value, 6, 0, 1, 1);

        labelJoint1Value = new QLabel(tab_targets);
        labelJoint1Value->setObjectName(QString::fromUtf8("labelJoint1Value"));

        jointValuesLayout->addWidget(labelJoint1Value, 2, 0, 1, 1);

        labelJoint6Value = new QLabel(tab_targets);
        labelJoint6Value->setObjectName(QString::fromUtf8("labelJoint6Value"));

        jointValuesLayout->addWidget(labelJoint6Value, 7, 0, 1, 1);

        valueJoint1 = new QLabel(tab_targets);
        valueJoint1->setObjectName(QString::fromUtf8("valueJoint1"));

        jointValuesLayout->addWidget(valueJoint1, 2, 1, 1, 1);

        valueJoint3 = new QLabel(tab_targets);
        valueJoint3->setObjectName(QString::fromUtf8("valueJoint3"));

        jointValuesLayout->addWidget(valueJoint3, 4, 1, 1, 1);

        valueJoint5 = new QLabel(tab_targets);
        valueJoint5->setObjectName(QString::fromUtf8("valueJoint5"));

        jointValuesLayout->addWidget(valueJoint5, 6, 1, 1, 1);

        valueJoint2 = new QLabel(tab_targets);
        valueJoint2->setObjectName(QString::fromUtf8("valueJoint2"));

        jointValuesLayout->addWidget(valueJoint2, 3, 1, 1, 1);

        valueJoint4 = new QLabel(tab_targets);
        valueJoint4->setObjectName(QString::fromUtf8("valueJoint4"));

        jointValuesLayout->addWidget(valueJoint4, 5, 1, 1, 1);

        valueJoint6 = new QLabel(tab_targets);
        valueJoint6->setObjectName(QString::fromUtf8("valueJoint6"));

        jointValuesLayout->addWidget(valueJoint6, 7, 1, 1, 1);


        verticalLayout_3->addLayout(jointValuesLayout);

        horizontalSpacer = new QSpacerItem(393, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        verticalLayout_3->addItem(horizontalSpacer);

        frame = new QFrame(tab_targets);
        frame->setObjectName(QString::fromUtf8("frame"));
        jointValuesLayout_3 = new QGridLayout(frame);
        jointValuesLayout_3->setObjectName(QString::fromUtf8("jointValuesLayout_3"));
        valueJoint6_3 = new QLabel(frame);
        valueJoint6_3->setObjectName(QString::fromUtf8("valueJoint6_3"));

        jointValuesLayout_3->addWidget(valueJoint6_3, 5, 1, 1, 1);

        labelJoint3Value_3 = new QLabel(frame);
        labelJoint3Value_3->setObjectName(QString::fromUtf8("labelJoint3Value_3"));

        jointValuesLayout_3->addWidget(labelJoint3Value_3, 2, 0, 1, 1);

        labelJoint5Value_3 = new QLabel(frame);
        labelJoint5Value_3->setObjectName(QString::fromUtf8("labelJoint5Value_3"));

        jointValuesLayout_3->addWidget(labelJoint5Value_3, 4, 0, 1, 1);

        labelJoint2Value_3 = new QLabel(frame);
        labelJoint2Value_3->setObjectName(QString::fromUtf8("labelJoint2Value_3"));

        jointValuesLayout_3->addWidget(labelJoint2Value_3, 1, 0, 1, 1);

        valueJoint1_3 = new QLabel(frame);
        valueJoint1_3->setObjectName(QString::fromUtf8("valueJoint1_3"));

        jointValuesLayout_3->addWidget(valueJoint1_3, 0, 1, 1, 1);

        valueJoint5_3 = new QLabel(frame);
        valueJoint5_3->setObjectName(QString::fromUtf8("valueJoint5_3"));

        jointValuesLayout_3->addWidget(valueJoint5_3, 4, 1, 1, 1);

        valueJoint3_3 = new QLabel(frame);
        valueJoint3_3->setObjectName(QString::fromUtf8("valueJoint3_3"));

        jointValuesLayout_3->addWidget(valueJoint3_3, 2, 1, 1, 1);

        labelJoint4Value_3 = new QLabel(frame);
        labelJoint4Value_3->setObjectName(QString::fromUtf8("labelJoint4Value_3"));

        jointValuesLayout_3->addWidget(labelJoint4Value_3, 3, 0, 1, 1);

        valueJoint4_3 = new QLabel(frame);
        valueJoint4_3->setObjectName(QString::fromUtf8("valueJoint4_3"));

        jointValuesLayout_3->addWidget(valueJoint4_3, 3, 1, 1, 1);

        labelJoint6Value_3 = new QLabel(frame);
        labelJoint6Value_3->setObjectName(QString::fromUtf8("labelJoint6Value_3"));

        jointValuesLayout_3->addWidget(labelJoint6Value_3, 5, 0, 1, 1);

        labelJoint1Value_3 = new QLabel(frame);
        labelJoint1Value_3->setObjectName(QString::fromUtf8("labelJoint1Value_3"));

        jointValuesLayout_3->addWidget(labelJoint1Value_3, 0, 0, 1, 1);

        valueJoint2_3 = new QLabel(frame);
        valueJoint2_3->setObjectName(QString::fromUtf8("valueJoint2_3"));

        jointValuesLayout_3->addWidget(valueJoint2_3, 1, 1, 1, 1);


        verticalLayout_3->addWidget(frame);

        horizontalSpacer_2 = new QSpacerItem(393, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        verticalLayout_3->addItem(horizontalSpacer_2);

        btnStartProgram = new QPushButton(tab_targets);
        btnStartProgram->setObjectName(QString::fromUtf8("btnStartProgram"));

        verticalLayout_3->addWidget(btnStartProgram);

        btnSopProgram = new QPushButton(tab_targets);
        btnSopProgram->setObjectName(QString::fromUtf8("btnSopProgram"));

        verticalLayout_3->addWidget(btnSopProgram);

        textTargetValues = new QTextEdit(tab_targets);
        textTargetValues->setObjectName(QString::fromUtf8("textTargetValues"));
        textTargetValues->setReadOnly(true);

        verticalLayout_3->addWidget(textTargetValues);

        tabWidget->addTab(tab_targets, QString());

        verticalLayout->addWidget(tabWidget);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 436, 22));
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
        labelJoint5->setText(QCoreApplication::translate("MainWindow", "Joint 5:", nullptr));
        labelJoint4->setText(QCoreApplication::translate("MainWindow", "Joint 4:", nullptr));
        labelJoint6->setText(QCoreApplication::translate("MainWindow", "Joint 6:", nullptr));
        labelJoint1->setText(QCoreApplication::translate("MainWindow", "Joint 1:", nullptr));
        labelJoint3->setText(QCoreApplication::translate("MainWindow", "Joint 3:", nullptr));
        labelJoint2->setText(QCoreApplication::translate("MainWindow", "Joint 2:", nullptr));
        btnApplyJointPositions->setText(QCoreApplication::translate("MainWindow", "Apply Joint Positions", nullptr));
        groupBoxPoseControl->setTitle(QCoreApplication::translate("MainWindow", "Cartesian Pose Control", nullptr));
        labelRoll->setText(QCoreApplication::translate("MainWindow", "Roll:", nullptr));
        btnMoveToPose->setText(QCoreApplication::translate("MainWindow", "Move End-Effector", nullptr));
        labelZ->setText(QCoreApplication::translate("MainWindow", "Z:", nullptr));
        labelYaw->setText(QCoreApplication::translate("MainWindow", "Yaw:", nullptr));
        labelX->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        labelY->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        labelPitch->setText(QCoreApplication::translate("MainWindow", "Pitch:", nullptr));
        groupBoxJogControls->setTitle(QCoreApplication::translate("MainWindow", "Jog Controls", nullptr));
        btnJogYPlus->setText(QCoreApplication::translate("MainWindow", "Y+", nullptr));
        btnJogZPlus->setText(QCoreApplication::translate("MainWindow", "Z+", nullptr));
        btnJogPPlus->setText(QCoreApplication::translate("MainWindow", "Pitch+", nullptr));
        btnJogXMinus->setText(QCoreApplication::translate("MainWindow", "X-", nullptr));
        btnJogZMinus->setText(QCoreApplication::translate("MainWindow", "Z-", nullptr));
        btnServoOff->setText(QCoreApplication::translate("MainWindow", "Servo Off", nullptr));
        btnJogPMinus->setText(QCoreApplication::translate("MainWindow", "Pitch-", nullptr));
        btnJogRMinus->setText(QCoreApplication::translate("MainWindow", "Roll-", nullptr));
        btnJogXPlus->setText(QCoreApplication::translate("MainWindow", "X+", nullptr));
        btnJogYawPlus->setText(QCoreApplication::translate("MainWindow", "Yaw+", nullptr));
        btnJogRPlus->setText(QCoreApplication::translate("MainWindow", "Roll+", nullptr));
        btnServoOn->setText(QCoreApplication::translate("MainWindow", "Servo On", nullptr));
        btnJogYMinus->setText(QCoreApplication::translate("MainWindow", "Y-", nullptr));
        btnJogYawMinus->setText(QCoreApplication::translate("MainWindow", "Yaw-", nullptr));
        labelSpeed->setText(QCoreApplication::translate("MainWindow", "Speed (0.5x - 2x):", nullptr));
        labelCurrentSpeed->setText(QCoreApplication::translate("MainWindow", "1.0x", nullptr));
        labelBoxCount->setText(QCoreApplication::translate("MainWindow", "Boxes: 0", nullptr));
        labelActiveSpeed->setText(QCoreApplication::translate("MainWindow", "Speed: 1.0x", nullptr));
        labelErrors->setText(QCoreApplication::translate("MainWindow", "Errors: None", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_control), QCoreApplication::translate("MainWindow", "Control", nullptr));
        btnShowJointValues->setText(QCoreApplication::translate("MainWindow", "Show Joint Values", nullptr));
        btnShowPoseValues->setText(QCoreApplication::translate("MainWindow", "Show Pose Values", nullptr));
        lineEditTargetName->setPlaceholderText(QCoreApplication::translate("MainWindow", "Enter target name...", nullptr));
        btnGoToTarget->setText(QCoreApplication::translate("MainWindow", "Go to Target", nullptr));
        btnSaveTarget->setText(QCoreApplication::translate("MainWindow", "Save joint Target", nullptr));
        btnSaveTarget_2->setText(QCoreApplication::translate("MainWindow", "Save Pose Target", nullptr));
        labelJoint2Value->setText(QCoreApplication::translate("MainWindow", "Joint2:", nullptr));
        labelJoint3Value->setText(QCoreApplication::translate("MainWindow", "Joint3:", nullptr));
        labelJoint4Value->setText(QCoreApplication::translate("MainWindow", "Joint4:", nullptr));
        labelJoint5Value->setText(QCoreApplication::translate("MainWindow", "Joint5:", nullptr));
        labelJoint1Value->setText(QCoreApplication::translate("MainWindow", "Joint1:", nullptr));
        labelJoint6Value->setText(QCoreApplication::translate("MainWindow", "Joint6:", nullptr));
        valueJoint1->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        valueJoint3->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        valueJoint5->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        valueJoint2->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        valueJoint4->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        valueJoint6->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        valueJoint6_3->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint3Value_3->setText(QCoreApplication::translate("MainWindow", "Z:", nullptr));
        labelJoint5Value_3->setText(QCoreApplication::translate("MainWindow", "Pitch:", nullptr));
        labelJoint2Value_3->setText(QCoreApplication::translate("MainWindow", "Y:", nullptr));
        valueJoint1_3->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        valueJoint5_3->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        valueJoint3_3->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint4Value_3->setText(QCoreApplication::translate("MainWindow", "Roll:", nullptr));
        valueJoint4_3->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint6Value_3->setText(QCoreApplication::translate("MainWindow", "Yaw:", nullptr));
        labelJoint1Value_3->setText(QCoreApplication::translate("MainWindow", "X:", nullptr));
        valueJoint2_3->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        btnStartProgram->setText(QCoreApplication::translate("MainWindow", "Start Program", nullptr));
        btnSopProgram->setText(QCoreApplication::translate("MainWindow", "Stop Progarm", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_targets), QCoreApplication::translate("MainWindow", "Targets", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINW_H
