#ifndef PLOTWINDOW_H
#define PLOTWINDOW_H

#include <QWidget>

namespace Ui {
class plotWindow;
}

class plotWindow : public QWidget
{
    Q_OBJECT

public:
    explicit plotWindow(QWidget *parent = 0);
    ~plotWindow();

private:
    Ui::plotWindow *ui;
};

#endif // PLOTWINDOW_H
