#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"
#include "ui_mainwindow.h"
#include "opencv2/opencv.hpp"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


    void addPoint(double x, int y); // 增加数据
    void clearData();     // 清楚数据
    void plot(); // 画曲线


private slots:
    void on_btn_add_clicked();

    void on_ptn_clear_clicked();

    void clickedGreph(QMouseEvent *event);
    void on_btn_stop_clicked();

    void on_btn_start_clicked();

private:
    Ui::MainWindow *ui;

    QVector<double> qv_x, qv_y, qv_t1, qv_t2;
    double programe_start_time;
    int history_num_ = 5000;
    int range_num_ = 5;
    bool mouse_flag_ = 0;
    bool stop_plot_flag_ = 0;
};



#endif // MAINWINDOW_H
