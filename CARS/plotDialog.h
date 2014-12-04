#ifndef PLOTDIALOG_H
#define PLOTDIALOG_H

#include <QDialog>

namespace Ui {
class plotDialog;
}

class plotDialog : public QDialog
{
    Q_OBJECT

public:
    explicit plotDialog(QWidget *parent = 0);
    ~plotDialog();

    void makePlot(int lengthX);

private:
    Ui::plotDialog *ui;
};

#endif // PLOTDIALOG_H
