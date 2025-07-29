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
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
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
    QHBoxLayout *jogControls;
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
    QPushButton *btnJogYPlus1;
    QPushButton *btnJogYMinus1;
    QHBoxLayout *servoControls;
    QPushButton *btnServoOn;
    QPushButton *btnServoOff;
    QHBoxLayout *speedControl;
    QLabel *labelSpeed;
    QSlider *sliderSpeed;
    QHBoxLayout *statusDisplay;
    QLabel *labelBoxCount;
    QLabel *labelCurrentSpeed;
    QLabel *labelErrors;
    QTextEdit *textLog;
    QWidget *tab_targets;
    QVBoxLayout *targetLayout;
    QLineEdit *lineEditTargetName;
    QHBoxLayout *targetButtons;
    QPushButton *btnSaveJointTarget;
    QPushButton *btnSavePoseTarget;
    QHBoxLayout *targetDropdownLayout;
    QComboBox *comboBoxTargets;
    QPushButton *btnGoToTarget;
    QHBoxLayout *targetDisplayButtons;
    QPushButton *btnShowJointValues;
    QPushButton *btnShowPoseValues;
    QGridLayout *jointValuesLayout;
    QLabel *labelJoint1;
    QLabel *valueJoint1;
    QLabel *labelJoint2;
    QLabel *valueJoint2;
    QLabel *labelJoint3;
    QLabel *valueJoint3;
    QLabel *labelJoint4;
    QLabel *valueJoint4;
    QLabel *labelJoint5;
    QLabel *valueJoint5;
    QLabel *labelJoint6;
    QLabel *valueJoint6;
    QTextEdit *textTargetValues;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1000, 700);
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
        jogControls = new QHBoxLayout();
        jogControls->setObjectName(QString::fromUtf8("jogControls"));
        gridLayoutJog = new QGridLayout();
        gridLayoutJog->setObjectName(QString::fromUtf8("gridLayoutJog"));
        btnJogXPlus = new QPushButton(tab_control);
        btnJogXPlus->setObjectName(QString::fromUtf8("btnJogXPlus"));

        gridLayoutJog->addWidget(btnJogXPlus, 0, 0, 1, 1);

        btnJogXMinus = new QPushButton(tab_control);
        btnJogXMinus->setObjectName(QString::fromUtf8("btnJogXMinus"));

        gridLayoutJog->addWidget(btnJogXMinus, 0, 1, 1, 1);

        btnJogYPlus = new QPushButton(tab_control);
        btnJogYPlus->setObjectName(QString::fromUtf8("btnJogYPlus"));

        gridLayoutJog->addWidget(btnJogYPlus, 1, 0, 1, 1);

        btnJogYMinus = new QPushButton(tab_control);
        btnJogYMinus->setObjectName(QString::fromUtf8("btnJogYMinus"));

        gridLayoutJog->addWidget(btnJogYMinus, 1, 1, 1, 1);

        btnJogZPlus = new QPushButton(tab_control);
        btnJogZPlus->setObjectName(QString::fromUtf8("btnJogZPlus"));

        gridLayoutJog->addWidget(btnJogZPlus, 2, 0, 1, 1);

        btnJogZMinus = new QPushButton(tab_control);
        btnJogZMinus->setObjectName(QString::fromUtf8("btnJogZMinus"));

        gridLayoutJog->addWidget(btnJogZMinus, 2, 1, 1, 1);

        btnJogRPlus = new QPushButton(tab_control);
        btnJogRPlus->setObjectName(QString::fromUtf8("btnJogRPlus"));

        gridLayoutJog->addWidget(btnJogRPlus, 3, 0, 1, 1);

        btnJogRMinus = new QPushButton(tab_control);
        btnJogRMinus->setObjectName(QString::fromUtf8("btnJogRMinus"));

        gridLayoutJog->addWidget(btnJogRMinus, 3, 1, 1, 1);

        btnJogPPlus = new QPushButton(tab_control);
        btnJogPPlus->setObjectName(QString::fromUtf8("btnJogPPlus"));

        gridLayoutJog->addWidget(btnJogPPlus, 4, 0, 1, 1);

        btnJogPMinus = new QPushButton(tab_control);
        btnJogPMinus->setObjectName(QString::fromUtf8("btnJogPMinus"));

        gridLayoutJog->addWidget(btnJogPMinus, 4, 1, 1, 1);

        btnJogYPlus1 = new QPushButton(tab_control);
        btnJogYPlus1->setObjectName(QString::fromUtf8("btnJogYPlus1"));

        gridLayoutJog->addWidget(btnJogYPlus1, 5, 0, 1, 1);

        btnJogYMinus1 = new QPushButton(tab_control);
        btnJogYMinus1->setObjectName(QString::fromUtf8("btnJogYMinus1"));

        gridLayoutJog->addWidget(btnJogYMinus1, 5, 1, 1, 1);


        jogControls->addLayout(gridLayoutJog);


        controlLayout->addLayout(jogControls);

        servoControls = new QHBoxLayout();
        servoControls->setObjectName(QString::fromUtf8("servoControls"));
        btnServoOn = new QPushButton(tab_control);
        btnServoOn->setObjectName(QString::fromUtf8("btnServoOn"));

        servoControls->addWidget(btnServoOn);

        btnServoOff = new QPushButton(tab_control);
        btnServoOff->setObjectName(QString::fromUtf8("btnServoOff"));

        servoControls->addWidget(btnServoOff);


        controlLayout->addLayout(servoControls);

        speedControl = new QHBoxLayout();
        speedControl->setObjectName(QString::fromUtf8("speedControl"));
        labelSpeed = new QLabel(tab_control);
        labelSpeed->setObjectName(QString::fromUtf8("labelSpeed"));

        speedControl->addWidget(labelSpeed);

        sliderSpeed = new QSlider(tab_control);
        sliderSpeed->setObjectName(QString::fromUtf8("sliderSpeed"));
        sliderSpeed->setMinimum(50);
        sliderSpeed->setMaximum(200);
        sliderSpeed->setOrientation(Qt::Horizontal);

        speedControl->addWidget(sliderSpeed);


        controlLayout->addLayout(speedControl);

        statusDisplay = new QHBoxLayout();
        statusDisplay->setObjectName(QString::fromUtf8("statusDisplay"));
        labelBoxCount = new QLabel(tab_control);
        labelBoxCount->setObjectName(QString::fromUtf8("labelBoxCount"));

        statusDisplay->addWidget(labelBoxCount);

        labelCurrentSpeed = new QLabel(tab_control);
        labelCurrentSpeed->setObjectName(QString::fromUtf8("labelCurrentSpeed"));

        statusDisplay->addWidget(labelCurrentSpeed);

        labelErrors = new QLabel(tab_control);
        labelErrors->setObjectName(QString::fromUtf8("labelErrors"));

        statusDisplay->addWidget(labelErrors);


        controlLayout->addLayout(statusDisplay);

        textLog = new QTextEdit(tab_control);
        textLog->setObjectName(QString::fromUtf8("textLog"));
        textLog->setReadOnly(true);

        controlLayout->addWidget(textLog);

        tabWidget->addTab(tab_control, QString());
        tab_targets = new QWidget();
        tab_targets->setObjectName(QString::fromUtf8("tab_targets"));
        targetLayout = new QVBoxLayout(tab_targets);
        targetLayout->setObjectName(QString::fromUtf8("targetLayout"));
        lineEditTargetName = new QLineEdit(tab_targets);
        lineEditTargetName->setObjectName(QString::fromUtf8("lineEditTargetName"));

        targetLayout->addWidget(lineEditTargetName);

        targetButtons = new QHBoxLayout();
        targetButtons->setObjectName(QString::fromUtf8("targetButtons"));
        btnSaveJointTarget = new QPushButton(tab_targets);
        btnSaveJointTarget->setObjectName(QString::fromUtf8("btnSaveJointTarget"));

        targetButtons->addWidget(btnSaveJointTarget);

        btnSavePoseTarget = new QPushButton(tab_targets);
        btnSavePoseTarget->setObjectName(QString::fromUtf8("btnSavePoseTarget"));

        targetButtons->addWidget(btnSavePoseTarget);


        targetLayout->addLayout(targetButtons);

        targetDropdownLayout = new QHBoxLayout();
        targetDropdownLayout->setObjectName(QString::fromUtf8("targetDropdownLayout"));
        comboBoxTargets = new QComboBox(tab_targets);
        comboBoxTargets->setObjectName(QString::fromUtf8("comboBoxTargets"));

        targetDropdownLayout->addWidget(comboBoxTargets);

        btnGoToTarget = new QPushButton(tab_targets);
        btnGoToTarget->setObjectName(QString::fromUtf8("btnGoToTarget"));

        targetDropdownLayout->addWidget(btnGoToTarget);


        targetLayout->addLayout(targetDropdownLayout);

        targetDisplayButtons = new QHBoxLayout();
        targetDisplayButtons->setObjectName(QString::fromUtf8("targetDisplayButtons"));
        btnShowJointValues = new QPushButton(tab_targets);
        btnShowJointValues->setObjectName(QString::fromUtf8("btnShowJointValues"));

        targetDisplayButtons->addWidget(btnShowJointValues);

        btnShowPoseValues = new QPushButton(tab_targets);
        btnShowPoseValues->setObjectName(QString::fromUtf8("btnShowPoseValues"));

        targetDisplayButtons->addWidget(btnShowPoseValues);


        targetLayout->addLayout(targetDisplayButtons);

        jointValuesLayout = new QGridLayout();
        jointValuesLayout->setObjectName(QString::fromUtf8("jointValuesLayout"));
        labelJoint1 = new QLabel(tab_targets);
        labelJoint1->setObjectName(QString::fromUtf8("labelJoint1"));

        jointValuesLayout->addWidget(labelJoint1, 0, 0, 1, 1);

        valueJoint1 = new QLabel(tab_targets);
        valueJoint1->setObjectName(QString::fromUtf8("valueJoint1"));

        jointValuesLayout->addWidget(valueJoint1, 0, 1, 1, 1);

        labelJoint2 = new QLabel(tab_targets);
        labelJoint2->setObjectName(QString::fromUtf8("labelJoint2"));

        jointValuesLayout->addWidget(labelJoint2, 1, 0, 1, 1);

        valueJoint2 = new QLabel(tab_targets);
        valueJoint2->setObjectName(QString::fromUtf8("valueJoint2"));

        jointValuesLayout->addWidget(valueJoint2, 1, 1, 1, 1);

        labelJoint3 = new QLabel(tab_targets);
        labelJoint3->setObjectName(QString::fromUtf8("labelJoint3"));

        jointValuesLayout->addWidget(labelJoint3, 2, 0, 1, 1);

        valueJoint3 = new QLabel(tab_targets);
        valueJoint3->setObjectName(QString::fromUtf8("valueJoint3"));

        jointValuesLayout->addWidget(valueJoint3, 2, 1, 1, 1);

        labelJoint4 = new QLabel(tab_targets);
        labelJoint4->setObjectName(QString::fromUtf8("labelJoint4"));

        jointValuesLayout->addWidget(labelJoint4, 3, 0, 1, 1);

        valueJoint4 = new QLabel(tab_targets);
        valueJoint4->setObjectName(QString::fromUtf8("valueJoint4"));

        jointValuesLayout->addWidget(valueJoint4, 3, 1, 1, 1);

        labelJoint5 = new QLabel(tab_targets);
        labelJoint5->setObjectName(QString::fromUtf8("labelJoint5"));

        jointValuesLayout->addWidget(labelJoint5, 4, 0, 1, 1);

        valueJoint5 = new QLabel(tab_targets);
        valueJoint5->setObjectName(QString::fromUtf8("valueJoint5"));

        jointValuesLayout->addWidget(valueJoint5, 4, 1, 1, 1);

        labelJoint6 = new QLabel(tab_targets);
        labelJoint6->setObjectName(QString::fromUtf8("labelJoint6"));

        jointValuesLayout->addWidget(labelJoint6, 5, 0, 1, 1);

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

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Parol6 Robotic Arm Control", nullptr));
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
        btnJogYPlus1->setText(QCoreApplication::translate("MainWindow", "Yaw+", nullptr));
        btnJogYMinus1->setText(QCoreApplication::translate("MainWindow", "Yaw-", nullptr));
        btnServoOn->setText(QCoreApplication::translate("MainWindow", "Servo ON", nullptr));
        btnServoOff->setText(QCoreApplication::translate("MainWindow", "Servo OFF", nullptr));
        labelSpeed->setText(QCoreApplication::translate("MainWindow", "Speed (0.5x - 2x):", nullptr));
        labelBoxCount->setText(QCoreApplication::translate("MainWindow", "Boxes: 0", nullptr));
        labelCurrentSpeed->setText(QCoreApplication::translate("MainWindow", "Speed: 1.0x", nullptr));
        labelErrors->setText(QCoreApplication::translate("MainWindow", "Errors: None", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_control), QCoreApplication::translate("MainWindow", "Control", nullptr));
        lineEditTargetName->setPlaceholderText(QCoreApplication::translate("MainWindow", "Enter target name...", nullptr));
        btnSaveJointTarget->setText(QCoreApplication::translate("MainWindow", "Save Joint Target", nullptr));
        btnSavePoseTarget->setText(QCoreApplication::translate("MainWindow", "Save Pose Target", nullptr));
        btnGoToTarget->setText(QCoreApplication::translate("MainWindow", "Go to Target", nullptr));
        btnShowJointValues->setText(QCoreApplication::translate("MainWindow", "Show Joint Values", nullptr));
        btnShowPoseValues->setText(QCoreApplication::translate("MainWindow", "Show Pose Values", nullptr));
        labelJoint1->setText(QCoreApplication::translate("MainWindow", "Joint 1:", nullptr));
        valueJoint1->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint2->setText(QCoreApplication::translate("MainWindow", "Joint 2:", nullptr));
        valueJoint2->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint3->setText(QCoreApplication::translate("MainWindow", "Joint 3:", nullptr));
        valueJoint3->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint4->setText(QCoreApplication::translate("MainWindow", "Joint 4:", nullptr));
        valueJoint4->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint5->setText(QCoreApplication::translate("MainWindow", "Joint 5:", nullptr));
        valueJoint5->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        labelJoint6->setText(QCoreApplication::translate("MainWindow", "Joint 6:", nullptr));
        valueJoint6->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_targets), QCoreApplication::translate("MainWindow", "Targets", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
