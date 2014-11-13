/********************************************************************************
** Form generated from reading UI file 'cargroupbox.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CARGROUPBOX_H
#define UI_CARGROUPBOX_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CarGroupBox
{
public:
    QGridLayout *gridLayout;
    QLabel *label;
    QLabel *idLabel;
    QLabel *label_6;
    QLabel *label_2;
    QComboBox *filterComboBox;
    QLabel *label_3;
    QComboBox *motionModelComboBox;
    QLabel *label_4;
    QComboBox *controllerComboBox;
    QRadioButton *assistedRadioButton;
    QRadioButton *autoRadioButton;
    QRadioButton *manualRadioButton;
    QRadioButton *notConnectedRadioButton;

    void setupUi(QWidget *CarGroupBox)
    {
        if (CarGroupBox->objectName().isEmpty())
            CarGroupBox->setObjectName(QStringLiteral("CarGroupBox"));
        CarGroupBox->resize(408, 80);
        gridLayout = new QGridLayout(CarGroupBox);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label = new QLabel(CarGroupBox);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);

        gridLayout->addWidget(label, 0, 0, 1, 1);

        idLabel = new QLabel(CarGroupBox);
        idLabel->setObjectName(QStringLiteral("idLabel"));

        gridLayout->addWidget(idLabel, 0, 1, 1, 1);

        label_6 = new QLabel(CarGroupBox);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout->addWidget(label_6, 1, 0, 1, 1);

        label_2 = new QLabel(CarGroupBox);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        filterComboBox = new QComboBox(CarGroupBox);
        filterComboBox->setObjectName(QStringLiteral("filterComboBox"));
        filterComboBox->setEnabled(false);

        gridLayout->addWidget(filterComboBox, 2, 1, 1, 1);

        label_3 = new QLabel(CarGroupBox);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 2, 2, 1, 1);

        motionModelComboBox = new QComboBox(CarGroupBox);
        motionModelComboBox->setObjectName(QStringLiteral("motionModelComboBox"));
        motionModelComboBox->setEnabled(false);

        gridLayout->addWidget(motionModelComboBox, 2, 3, 1, 1);

        label_4 = new QLabel(CarGroupBox);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 2, 5, 1, 1);

        controllerComboBox = new QComboBox(CarGroupBox);
        controllerComboBox->setObjectName(QStringLiteral("controllerComboBox"));
        controllerComboBox->setEnabled(false);

        gridLayout->addWidget(controllerComboBox, 2, 6, 1, 1);

        assistedRadioButton = new QRadioButton(CarGroupBox);
        assistedRadioButton->setObjectName(QStringLiteral("assistedRadioButton"));

        gridLayout->addWidget(assistedRadioButton, 1, 2, 1, 1);

        autoRadioButton = new QRadioButton(CarGroupBox);
        autoRadioButton->setObjectName(QStringLiteral("autoRadioButton"));
        autoRadioButton->setEnabled(true);
        autoRadioButton->setChecked(false);
        autoRadioButton->setAutoExclusive(false);

        gridLayout->addWidget(autoRadioButton, 1, 1, 1, 1);

        manualRadioButton = new QRadioButton(CarGroupBox);
        manualRadioButton->setObjectName(QStringLiteral("manualRadioButton"));
        manualRadioButton->setEnabled(true);
        manualRadioButton->setChecked(true);
        manualRadioButton->setAutoExclusive(false);

        gridLayout->addWidget(manualRadioButton, 1, 3, 1, 1);

        notConnectedRadioButton = new QRadioButton(CarGroupBox);
        notConnectedRadioButton->setObjectName(QStringLiteral("notConnectedRadioButton"));

        gridLayout->addWidget(notConnectedRadioButton, 1, 6, 1, 1);


        retranslateUi(CarGroupBox);

        QMetaObject::connectSlotsByName(CarGroupBox);
    } // setupUi

    void retranslateUi(QWidget *CarGroupBox)
    {
        CarGroupBox->setWindowTitle(QApplication::translate("CarGroupBox", "Form", 0));
        label->setText(QApplication::translate("CarGroupBox", "Car ID:", 0));
        idLabel->setText(QApplication::translate("CarGroupBox", "0", 0));
        label_6->setText(QApplication::translate("CarGroupBox", "Mode:", 0));
        label_2->setText(QApplication::translate("CarGroupBox", "Filter:", 0));
        label_3->setText(QApplication::translate("CarGroupBox", "Motion Model:", 0));
        label_4->setText(QApplication::translate("CarGroupBox", "Controller:", 0));
        assistedRadioButton->setText(QApplication::translate("CarGroupBox", "Assisted", 0));
        autoRadioButton->setText(QApplication::translate("CarGroupBox", "Auto", 0));
        manualRadioButton->setText(QApplication::translate("CarGroupBox", "Manual", 0));
        notConnectedRadioButton->setText(QApplication::translate("CarGroupBox", "Not Connected", 0));
    } // retranslateUi

};

namespace Ui {
    class CarGroupBox: public Ui_CarGroupBox {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CARGROUPBOX_H
