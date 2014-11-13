/********************************************************************************
** Form generated from reading UI file 'carsettingsdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CARSETTINGSDIALOG_H
#define UI_CARSETTINGSDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CarSettingsDialog
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *label_22;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *scrollAreaContentsLayout;
    QHBoxLayout *horizontalLayout;
    QPushButton *addCarButton;
    QPushButton *removeCarButton;
    QSpacerItem *horizontalSpacer;
    QPushButton *closeButton;

    void setupUi(QDialog *CarSettingsDialog)
    {
        if (CarSettingsDialog->objectName().isEmpty())
            CarSettingsDialog->setObjectName(QStringLiteral("CarSettingsDialog"));
        CarSettingsDialog->resize(542, 168);
        verticalLayout = new QVBoxLayout(CarSettingsDialog);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        label_22 = new QLabel(CarSettingsDialog);
        label_22->setObjectName(QStringLiteral("label_22"));

        verticalLayout->addWidget(label_22);

        scrollArea = new QScrollArea(CarSettingsDialog);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(scrollArea->sizePolicy().hasHeightForWidth());
        scrollArea->setSizePolicy(sizePolicy);
        scrollArea->setFrameShadow(QFrame::Plain);
        scrollArea->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        scrollArea->setWidgetResizable(true);
        scrollArea->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 522, 98));
        scrollAreaContentsLayout = new QVBoxLayout(scrollAreaWidgetContents);
        scrollAreaContentsLayout->setObjectName(QStringLiteral("scrollAreaContentsLayout"));
        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout->addWidget(scrollArea);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        addCarButton = new QPushButton(CarSettingsDialog);
        addCarButton->setObjectName(QStringLiteral("addCarButton"));

        horizontalLayout->addWidget(addCarButton);

        removeCarButton = new QPushButton(CarSettingsDialog);
        removeCarButton->setObjectName(QStringLiteral("removeCarButton"));
        removeCarButton->setEnabled(false);

        horizontalLayout->addWidget(removeCarButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        closeButton = new QPushButton(CarSettingsDialog);
        closeButton->setObjectName(QStringLiteral("closeButton"));

        horizontalLayout->addWidget(closeButton);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(CarSettingsDialog);

        QMetaObject::connectSlotsByName(CarSettingsDialog);
    } // setupUi

    void retranslateUi(QDialog *CarSettingsDialog)
    {
        CarSettingsDialog->setWindowTitle(QApplication::translate("CarSettingsDialog", "Car Settings", 0));
        label_22->setText(QApplication::translate("CarSettingsDialog", "Caution: do not connect more cars than there are output channels available on the NI-DAQ device.", 0));
        addCarButton->setText(QApplication::translate("CarSettingsDialog", "Add Car", 0));
        removeCarButton->setText(QApplication::translate("CarSettingsDialog", "Remove Car", 0));
        closeButton->setText(QApplication::translate("CarSettingsDialog", "Close", 0));
    } // retranslateUi

};

namespace Ui {
    class CarSettingsDialog: public Ui_CarSettingsDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CARSETTINGSDIALOG_H
