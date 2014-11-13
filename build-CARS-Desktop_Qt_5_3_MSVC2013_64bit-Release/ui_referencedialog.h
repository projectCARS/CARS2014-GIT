/********************************************************************************
** Form generated from reading UI file 'referencedialog.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_REFERENCEDIALOG_H
#define UI_REFERENCEDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_ReferenceDialog
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QLabel *nameLabel;
    QSpacerItem *horizontalSpacer_2;
    QLabel *imageLabel;
    QFrame *line;
    QHBoxLayout *horizontalLayout;
    QPushButton *chooseFileButton;
    QSpacerItem *horizontalSpacer;
    QPushButton *closeButton;

    void setupUi(QDialog *ReferenceDialog)
    {
        if (ReferenceDialog->objectName().isEmpty())
            ReferenceDialog->setObjectName(QStringLiteral("ReferenceDialog"));
        ReferenceDialog->resize(600, 550);
        verticalLayout = new QVBoxLayout(ReferenceDialog);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label = new QLabel(ReferenceDialog);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_2->addWidget(label);

        nameLabel = new QLabel(ReferenceDialog);
        nameLabel->setObjectName(QStringLiteral("nameLabel"));

        horizontalLayout_2->addWidget(nameLabel);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout_2);

        imageLabel = new QLabel(ReferenceDialog);
        imageLabel->setObjectName(QStringLiteral("imageLabel"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(imageLabel->sizePolicy().hasHeightForWidth());
        imageLabel->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(imageLabel);

        line = new QFrame(ReferenceDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        chooseFileButton = new QPushButton(ReferenceDialog);
        chooseFileButton->setObjectName(QStringLiteral("chooseFileButton"));

        horizontalLayout->addWidget(chooseFileButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        closeButton = new QPushButton(ReferenceDialog);
        closeButton->setObjectName(QStringLiteral("closeButton"));

        horizontalLayout->addWidget(closeButton);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(ReferenceDialog);

        QMetaObject::connectSlotsByName(ReferenceDialog);
    } // setupUi

    void retranslateUi(QDialog *ReferenceDialog)
    {
        ReferenceDialog->setWindowTitle(QApplication::translate("ReferenceDialog", "Reference Settings", 0));
        label->setText(QApplication::translate("ReferenceDialog", "Current reference curve:", 0));
        nameLabel->setText(QApplication::translate("ReferenceDialog", "default_reference.txt", 0));
        imageLabel->setText(QString());
        chooseFileButton->setText(QApplication::translate("ReferenceDialog", "Choose File ", 0));
        closeButton->setText(QApplication::translate("ReferenceDialog", "Close", 0));
    } // retranslateUi

};

namespace Ui {
    class ReferenceDialog: public Ui_ReferenceDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REFERENCEDIALOG_H
