#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->plot->addGraph();
    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssCross);
    ui->plot->graph(0)->setLineStyle(QCPGraph::lsLine);
    ui->plot->graph(0)->setPen(QColor(170, 20, 240));

    ui->plot->addGraph();
    ui->plot->graph(1)->setScatterStyle(QCPScatterStyle::ssCross);
    ui->plot->graph(1)->setLineStyle(QCPGraph::lsLine);
    ui->plot->graph(1)->setPen(QColor(255, 120, 0));
    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    connect(ui->plot, SIGNAL(mousePress(QMouseEvent*)), SLOT(clickedGreph(QMouseEvent*)));

    programe_start_time = cv::getTickCount();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::addPoint(double x, int y)
{
    double t_tmp = cv::getTickCount();
    double t = (t_tmp - programe_start_time)/cv::getTickFrequency();

    switch (y) {
    case 0:
        qv_x.push_back(x);
        qv_t1.push_back(t);
        break;

    case 1:
        qv_y.push_back(x);
        qv_t2.push_back(t);
        break;

    }
    if(qv_x.size()>history_num_)
    {
        qv_x.pop_front();
        qv_t1.pop_front();
    }

    if(qv_y.size()>history_num_)
    {
        qv_y.pop_front();
        qv_t2.pop_front();
    }

    if(mouse_flag_ == 0)
    {
        ui->plot->xAxis->rescale(); // set Range
        ui->plot->yAxis->rescale(); // set Range
        //    ui->plot->graph(0)->rescaleValueAxis(false, true);
        ui->plot->xAxis->setRange(ui->plot->xAxis->range().upper, range_num_, Qt::AlignRight);  // move
    }
}

void MainWindow::clearData()
{
    qv_x.clear();
    qv_y.clear();
    qv_t1.clear();
    qv_t2.clear();
}

void MainWindow::plot()
{
    if(!stop_plot_flag_)
    {
        ui->plot->graph(0)->setData(qv_t1, qv_x);
        ui->plot->graph(1)->setData(qv_t2, qv_y);
        ui->plot->replot();
//        ui->plot->update();
    }
    // ui->plot->xAxis->setRange(ui->plot->xAxis->range().upper, 100, Qt::AlignRight);
    // ui->plot->rescaleAxes();
}

void MainWindow::on_btn_add_clicked()
{
    addPoint(ui->bx_x->value(), ui->bx_y->value());
    plot();
}

void MainWindow::on_ptn_clear_clicked()
{
    clearData();
    plot();
}

void MainWindow::clickedGreph(QMouseEvent *event)
{
    QPoint point = event->pos();
    double m_x = ui->plot->xAxis->pixelToCoord(point.x());
    double m_y = ui->plot->yAxis->pixelToCoord(point.y());
    qDebug()<< m_x<< m_y;

    int button_flag = event->buttons();
    QVector<double> qv_tmp;
    if(button_flag == 1) // left button
    {
        mouse_flag_ = 1;
    }else
    {
        mouse_flag_ = 0;
    }
    //     addPoint(ui->plot->xAxis->pixelToCoord(point.x()),ui->plot->yAxis->pixelToCoord(point.y()));
    //     plot();
}





void MainWindow::on_btn_stop_clicked()
{
    stop_plot_flag_ = 1;
}

void MainWindow::on_btn_start_clicked()
{
    stop_plot_flag_ = 0;
}
