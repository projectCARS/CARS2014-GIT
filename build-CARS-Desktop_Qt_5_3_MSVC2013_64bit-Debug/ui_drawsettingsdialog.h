/********************************************************************************
** Form generated from reading UI file 'drawsettingsdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DRAWSETTINGSDIALOG_H
#define UI_DRAWSETTINGSDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_DrawSettingsDialog
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QCheckBox *animationsCheckBox;
    QLabel *label_3;
    QCheckBox *referenceCheckBox;
    QLabel *label_2;
    QLabel *label_9;
    QLabel *backgroundLabel;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout;
    QLabel *label_4;
    QCheckBox *pathCheckBox;
    QLabel *label_5;
    QCheckBox *circleCheckBox;
    QLabel *label_6;
    QCheckBox *rectangleCheckBox;
    QLabel *label_7;
    QComboBox *idComboBox;
    QLabel *label_8;
    QCheckBox *carTracksCheckBox;
    QHBoxLayout *horizontalLayout;
    QPushButton *chooseBackgroundButton;
    QSpacerItem *horizontalSpacer;
    QPushButton *closeButton;

    void setupUi(QDialog *DrawSettingsDialog)
    {
        if (DrawSettingsDialog->objectName().isEmpty())
            DrawSettingsDialog->setObjectName(QStringLiteral("DrawSettingsDialog"));
        DrawSettingsDialog->resize(324, 300);
        verticalLayout = new QVBoxLayout(DrawSettingsDialog);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        label = new QLabel(DrawSettingsDialog);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout->addWidget(label);

        groupBox = new QGroupBox(DrawSettingsDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        animationsCheckBox = new QCheckBox(groupBox);
        animationsCheckBox->setObjectName(QStringLiteral("animationsCheckBox"));

        gridLayout_2->addWidget(animationsCheckBox, 1, 1, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout_2->addWidget(label_3, 1, 0, 1, 1);

        referenceCheckBox = new QCheckBox(groupBox);
        referenceCheckBox->setObjectName(QStringLiteral("referenceCheckBox"));

        gridLayout_2->addWidget(referenceCheckBox, 0, 1, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout_2->addWidget(label_2, 0, 0, 1, 1);

        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QStringLiteral("label_9"));

        gridLayout_2->addWidget(label_9, 2, 0, 1, 1);

        backgroundLabel = new QLabel(groupBox);
        backgroundLabel->setObjectName(QStringLiteral("backgroundLabel"));

        gridLayout_2->addWidget(backgroundLabel, 2, 1, 1, 1);


        verticalLayout->addWidget(groupBox);

        groupBox_2 = new QGroupBox(DrawSettingsDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        gridLayout = new QGridLayout(groupBox_2);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 1, 0, 1, 1);

        pathCheckBox = new QCheckBox(groupBox_2);
        pathCheckBox->setObjectName(QStringLiteral("pathCheckBox"));

        gridLayout->addWidget(pathCheckBox, 1, 1, 1, 1);

        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout->addWidget(label_5, 2, 0, 1, 1);

        circleCheckBox = new QCheckBox(groupBox_2);
        circleCheckBox->setObjectName(QStringLiteral("circleCheckBox"));

        gridLayout->addWidget(circleCheckBox, 2, 1, 1, 1);

        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout->addWidget(label_6, 3, 0, 1, 1);

        rectangleCheckBox = new QCheckBox(groupBox_2);
        rectangleCheckBox->setObjectName(QStringLiteral("rectangleCheckBox"));

        gridLayout->addWidget(rectangleCheckBox, 3, 1, 1, 1);

        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout->addWidget(label_7, 0, 0, 1, 1);

        idComboBox = new QComboBox(groupBox_2);
        idComboBox->setObjectName(QStringLiteral("idComboBox"));

        gridLayout->addWidget(idComboBox, 0, 1, 1, 1);

        label_8 = new QLabel(groupBox_2);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout->addWidget(label_8, 4, 0, 1, 1);

        carTracksCheckBox = new QCheckBox(groupBox_2);
        carTracksCheckBox->setObjectName(QStringLiteral("carTracksCheckBox"));

        gridLayout->addWidget(carTracksCheckBox, 4, 1, 1, 1);


        verticalLayout->addWidget(groupBox_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        chooseBackgroundButton = new QPushButton(DrawSettingsDialog);
        chooseBackgroundButton->setObjectName(QStringLiteral("chooseBackgroundButton"));

        horizontalLayout->addWidget(chooseBackgroundButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        closeButton = new QPushButton(DrawSettingsDialog);
        closeButton->setObjectName(QStringLiteral("closeButton"));

        horizontalLayout->addWidget(closeButton);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(DrawSettingsDialog);

        QMetaObject::connectSlotsByName(DrawSettingsDialog);
    } // setupUi

    void retranslateUi(QDialog *DrawSettingsDialog)
    {
        DrawSettingsDialog->setWindowTitle(QApplication::translate("DrawSettingsDialog", "Draw Settings", 0));
        label->setText(QApplication::translate("DrawSettingsDialog", "Select features that should be drawn to the projector.\n"
"The background image is shown when animations are disabled.", 0));
        groupBox->setTitle(QApplication::translate("DrawSettingsDialog", "General settings", 0));
        animationsCheckBox->setText(QString());
        label_3->setText(QApplication::translate("DrawSettingsDialog", "Animations:", 0));
        referenceCheckBox->setText(QString());
        label_2->setText(QApplication::translate("DrawSettingsDialog", "Reference:", 0));
        label_9->setText(QApplication::translate("DrawSettingsDialog", "Background:", 0));
        backgroundLabel->setText(QString());
        groupBox_2->setTitle(QApplication::translate("DrawSettingsDialog", "Car specific settings", 0));
        label_4->setText(QApplication::translate("DrawSettingsDialog", "Path:", 0));
        pathCheckBox->setText(QString());
        label_5->setText(QApplication::translate("DrawSettingsDialog", "Circle:", 0));
        circleCheckBox->setText(QString());
        label_6->setText(QApplication::translate("DrawSettingsDialog", "Rectangle:", 0));
        rectangleCheckBox->setText(QString());
        label_7->setText(QApplication::translate("DrawSettingsDialog", "Car ID:", 0));
        label_8->setText(QApplication::translate("DrawSettingsDialog", "Car Tracks:", 0));
        carTracksCheckBox->setText(QString());
        chooseBackgroundButton->setText(QApplication::translate("DrawSettingsDialog", "Choose Background", 0));
        closeButton->setText(QApplication::translate("DrawSettingsDialog", "Close", 0));
    } // retranslateUi

};

namespace Ui {
    class DrawSettingsDialog: public Ui_DrawSettingsDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DRAWSETTINGSDIALOG_H
