/****************************************************************************\
 * Copyright (C) 2023 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include "CameraControlWidget.hpp"

#include <QHBoxLayout>

namespace pmd_royale_ros_examples {

CameraControlWidget::CameraControlWidget(std::shared_ptr<rclcpp::Node> node, std::string cameraNodeName,
                                         QWidget *parent)
    : QWidget(parent), CameraParametersClient(node, cameraNodeName), m_nh(node), m_isInit(false) {
    QVBoxLayout *controlLayout = new QVBoxLayout(this);
    // Use Case
    controlLayout->addWidget(new QLabel("Use Case:"));
    m_comboBoxUseCases = new QComboBox;
    controlLayout->addWidget(m_comboBoxUseCases);

    connect(m_comboBoxUseCases, SIGNAL(currentTextChanged(const QString)), this, SLOT(setUseCase(const QString)));
    connect(m_comboBoxUseCases, SIGNAL(currentTextChanged(const QString)), this, SLOT(setUseCase(const QString)));

    for (auto i = 0u; i < ROYALE_ROS_MAX_STREAMS; ++i) {
        controlLayout->addWidget(new QLabel(QString("Stream ") + QString::number(i) + " : "));

        // Exposure Time
        m_labelExpoTime[i] = new QLabel;
        controlLayout->addWidget(m_labelExpoTime[i]);
        QHBoxLayout *exTimeLayout = new QHBoxLayout;

        m_sliderExpoTime[i] = new QSlider(Qt::Horizontal);
        m_sliderExpoTime[i]->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        m_sliderExpoTime[i]->setTracking(false);
        exTimeLayout->addWidget(m_sliderExpoTime[i]);

        m_lineEditExpoTime[i] = new QLineEdit;
        m_lineEditExpoTime[i]->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
        exTimeLayout->addWidget(m_lineEditExpoTime[i]);

        controlLayout->addLayout(exTimeLayout);

        // Auto Exposure
        controlLayout->addWidget(new QLabel("Auto Exposure:"));
        m_checkBoxAutoExpo[i] = new QCheckBox;
        controlLayout->addWidget(m_checkBoxAutoExpo[i]);

        // Parameters
        controlLayout->addWidget(new QLabel("Parameter:"));
        m_lineEditParams[i] = new QLineEdit;
        controlLayout->addWidget(m_lineEditParams[i]);

        setLayout(controlLayout);

        std::map<std::string, std::vector<std::string>> topicInfos = node->get_topic_names_and_types();

        controlLayout->addWidget(new QLabel("Camera:"));
        m_cams = new QComboBox;

        std::set<std::string> cameras;

        for (auto curTopic : topicInfos) {
            if (curTopic.first.find("royale_cam_") != std::string::npos) {
                auto div = curTopic.first.find("/", 1);
                if (div != std::string::npos) {
                    auto camName = curTopic.first.substr(1, div - 1);
                    cameras.insert(camName);
                }
            }
        }

        for (auto curCam : cameras) {
            std::cout << "cam: " << curCam.c_str() << std::endl;
            m_cams->addItem(curCam.c_str());
        }

        controlLayout->addWidget(m_cams);

        connect(m_sliderExpoTime[i], &QSlider::valueChanged, this, [this, i](int val) { setExposureTime(val, i); });
        connect(m_checkBoxAutoExpo[i], &QCheckBox::toggled, this, [this, i](bool val) { setExposureMode(val, i); });
        connect(m_lineEditExpoTime[i], &QLineEdit::editingFinished, this, [this, i](void) { preciseExposureTimeSetting(i); });
        connect(m_lineEditParams[i], &QLineEdit::editingFinished, this, [this, i](void) { setProcParameter(i); });
        connect(m_cams, SIGNAL(currentIndexChanged(int)), this, SLOT(chooseCamera(int)));

        m_cams->currentIndexChanged(0);
    }
    subscribeForCameraParameters({"available_usecases", "usecase",
                                  "gray_image_divisor", "min_distance_filter", "max_distance_filter"});
    for (auto i = 0u; i < ROYALE_ROS_MAX_STREAMS; ++i) {
        subscribeForCameraParameters({"exposure_time_" + std::to_string(i), "auto_exposure_" + std::to_string(i)});
        m_pubParameters[i] = m_nh->create_publisher<std_msgs::msg::String>(
            cameraNodeName + "/proc_params_" + std::to_string(i), 10);
    }
}

CameraControlWidget::~CameraControlWidget() {}

void CameraControlWidget::chooseCamera(int idx) {
    std::string curCam = m_cams->itemText(idx).toStdString();

    // Advertise the topics to publish the message of the changed settings in the UI
    m_pubIsInit = m_nh->create_publisher<std_msgs::msg::String>(curCam + "/is_init", 1);
    m_pubUseCase = m_nh->create_publisher<std_msgs::msg::String>(curCam + "/use_case", 1);
    m_pubMinFilter = m_nh->create_publisher<std_msgs::msg::String>(curCam + "/min_filter", 1);
    m_pubMaxFilter = m_nh->create_publisher<std_msgs::msg::String>(curCam + "/max_filter", 1);
    m_pubDivisor = m_nh->create_publisher<std_msgs::msg::String>(curCam + "/divisor", 1);
    for (auto i = 0u; i < ROYALE_ROS_MAX_STREAMS; ++i) {
        m_pubExpoTime[i] = m_nh->create_publisher<std_msgs::msg::String>(std::string(curCam + "/expo_time_") + std::to_string(i), 1);
        m_pubExpoMode[i] = m_nh->create_publisher<std_msgs::msg::String>(std::string(curCam + "/expo_mode_") + std::to_string(i), 1);
        m_pubParams[i] = m_nh->create_publisher<std_msgs::msg::String>(std::string(curCam + "/params_") + std::to_string(i), 1);
    }

    // Subscribe the messages to show the changed state of the camera
    m_subInit = m_nh->create_subscription<std_msgs::msg::String>(curCam + "/init_panel", 1, std::bind(&CameraControlWidget::callbackInit, this, std::placeholders::_1));

    for (auto i = 0u; i < ROYALE_ROS_MAX_STREAMS; ++i) {
        std::function<void(const std_msgs::msg::String::SharedPtr msg)> f_expoP = std::bind(&CameraControlWidget::callbackExpoTimeParam, this, std::placeholders::_1, i);
        std::function<void(const std_msgs::msg::String::SharedPtr msg)> f_expoV = std::bind(&CameraControlWidget::callbackExpoTimeParam, this, std::placeholders::_1, i);
        std::function<void(const std_msgs::msg::String::SharedPtr msg)> f_fps = std::bind(&CameraControlWidget::callbackExpoTimeParam, this, std::placeholders::_1, i);

        m_subExpoTimeParam[i] = m_nh->create_subscription<std_msgs::msg::String>(curCam + "/expo_time_param_" + std::to_string(i), 1, f_expoP);
        m_subExpoTimeValue[i] = m_nh->create_subscription<std_msgs::msg::String>(curCam + "/expo_time_value_" + std::to_string(i), 1, f_expoV);
        m_subFps[i] = m_nh->create_subscription<std_msgs::msg::String>(curCam + "/update_fps_" + std::to_string(i), 1, f_fps);
    }

    // Publish the message to indicate that the initialization is complete
    m_isInit = false;
    std_msgs::msg::String msg;
    msg.data = m_isInit;
    m_pubIsInit->publish(msg);
}

void CameraControlWidget::callbackInit(const std_msgs::msg::String::SharedPtr msg) {
    if (!m_isInit) {
        QString str = msg->data.c_str();
        QStringList initList = str.split('/');

        m_currentUseCase = initList.at(0);

        float min = initList.at(1).toFloat();
        m_sliderMinFilter->blockSignals(true);
        m_sliderMinFilter->setValue((int)(min * 100));
        m_sliderMinFilter->blockSignals(false);
        m_lineEditMinFilter->setText(QString::number(min, 10, 2));

        float max = initList.at(2).toFloat();
        m_sliderMaxFilter->blockSignals(true);
        m_sliderMaxFilter->setValue((int)(max * 100));
        m_sliderMaxFilter->blockSignals(false);
        m_lineEditMaxFilter->setText(QString::number(max, 10, 2));

        m_sliderDivisor->blockSignals(true);
        m_sliderDivisor->setValue(initList.at(3).toInt());
        m_sliderDivisor->blockSignals(false);
        m_lineEditDivisor->setText(initList.at(3));

        m_comboBoxUseCases->blockSignals(true);
        m_comboBoxUseCases->clear();
        for (int i = 4; i < initList.size(); ++i) {
            m_comboBoxUseCases->addItem(initList.at(i));
        }
        auto cmbIdx = m_comboBoxUseCases->findText(m_currentUseCase);
        if (cmbIdx != -1) {
            m_comboBoxUseCases->setCurrentIndex(cmbIdx);
        }
        m_comboBoxUseCases->blockSignals(false);

        // Publish the message to indicate that the initialization is complete
        m_isInit = true;
        std_msgs::msg::String msg;
        msg.data = m_isInit;
        m_pubIsInit->publish(msg);
    }
}

void CameraControlWidget::callbackExpoTimeParam(const std_msgs::msg::String::SharedPtr msg, uint32_t streamIdx) {
    QString str = msg->data.c_str();
    m_minETSlider[streamIdx] = str.section('/', 0, 0).toInt();
    m_maxETSlider[streamIdx] = str.section('/', 1, 1).toInt();
    m_sliderExpoTime[streamIdx]->blockSignals(true);
    m_sliderExpoTime[streamIdx]->setRange(m_minETSlider[streamIdx], m_maxETSlider[streamIdx]);
    m_sliderExpoTime[streamIdx]->blockSignals(false);

    m_labelExpoTime[streamIdx]->setText("Exposure Time (" + QString::number(m_minETSlider[streamIdx]) + " - " + QString::number(m_maxETSlider[streamIdx]) + "):");

    m_isAutomatic[streamIdx] = str.section('/', 2, 2).toInt();
    m_checkBoxAutoExpo[streamIdx]->blockSignals(true);
    m_checkBoxAutoExpo[streamIdx]->setChecked(m_isAutomatic[streamIdx]);
    m_checkBoxAutoExpo[streamIdx]->blockSignals(false);
    m_lineEditExpoTime[streamIdx]->setEnabled(!m_isAutomatic[streamIdx]);
}

void CameraControlWidget::callbackExpoTimeValue(const std_msgs::msg::String::SharedPtr msg, uint32_t streamIdx) {
    int value = std::stoi(msg->data);
    m_sliderExpoTime[streamIdx]->blockSignals(true);
    m_sliderExpoTime[streamIdx]->setValue(value);
    m_sliderExpoTime[streamIdx]->blockSignals(false);

    m_exposureTime[streamIdx] = value;
    m_lineEditExpoTime[streamIdx]->setText(QString::number(m_exposureTime[streamIdx]));
}

void CameraControlWidget::callbackFps(const std_msgs::msg::String::SharedPtr msg, uint32_t streamIdx) {
    QString fps = msg->data.c_str();
    m_labelEditFps[streamIdx]->setText("Frames per Second: " + fps);
}

void CameraControlWidget::onNewCameraParameter(const CameraParameter &cameraParam) {
    auto param = cameraParam.parameter;
    auto descriptor = cameraParam.descriptor;

    if (param->get_name() == "available_usecases") {
        std::vector<std::string> availableUseCases = param->as_string_array();
        m_comboBoxUseCases->blockSignals(true);
        m_comboBoxUseCases->clear();
        for (auto useCase : availableUseCases) {
            m_comboBoxUseCases->addItem(QString::fromStdString(useCase));
        }
        m_comboBoxUseCases->blockSignals(false);
    } else if (param->get_name() == "usecase") {
        RCLCPP_INFO(m_nh->get_logger(), "Current usecase: %s", param->as_string().c_str());
        m_comboBoxUseCases->blockSignals(true);
        int currentIndex = m_comboBoxUseCases->findText(param->as_string().c_str());
        if (currentIndex != -1) {
            m_comboBoxUseCases->setCurrentIndex(currentIndex);
        }
        m_comboBoxUseCases->blockSignals(false);
    } else if (param->get_name().find("exposure_time_") == 0) {
        auto streamIdxStr = param->get_name().substr(strlen("exposure_time_"));
        auto streamIdx = stoi(streamIdxStr);
        auto exposureRange = descriptor->integer_range.front();
        m_sliderExpoTime[streamIdx]->blockSignals(true);
        m_sliderExpoTime[streamIdx]->setRange(exposureRange.from_value, exposureRange.to_value);
        m_labelExpoTime[streamIdx]->setText("Exposure Time (microseconds):");
        m_sliderExpoTime[streamIdx]->setValue(param->as_int());
        m_sliderExpoTime[streamIdx]->blockSignals(false);

        m_lineEditExpoTime[streamIdx]->blockSignals(true);
        m_lineEditExpoTime[streamIdx]->setText(QString::number(param->as_int()));
        m_lineEditExpoTime[streamIdx]->blockSignals(false);
    } else if (param->get_name().find("auto_exposure_") == 0) {
        auto streamIdxStr = param->get_name().substr(strlen("auto_exposure_"));
        auto streamIdx = stoi(streamIdxStr);
        m_checkBoxAutoExpo[streamIdx]->blockSignals(true);
        m_checkBoxAutoExpo[streamIdx]->setChecked(param->as_bool());
        m_sliderExpoTime[streamIdx]->setEnabled(!param->as_bool());
        m_labelExpoTime[streamIdx]->setEnabled(!param->as_bool());
        m_checkBoxAutoExpo[streamIdx]->blockSignals(false);
    }
}

void CameraControlWidget::setUseCase(const QString &currentMode) {
    rclcpp::Parameter parameter("usecase", currentMode.toStdString());
    setParameter(parameter);
}

void CameraControlWidget::setExposureTime(int value, uint32_t streamIdx) {
    rclcpp::Parameter parameter(std::string("exposure_time_") + std::to_string(streamIdx), value);
    setParameter(parameter);
}

void CameraControlWidget::setExposureMode(bool isAutomatic, uint32_t streamIdx) {
    rclcpp::Parameter parameter(std::string("auto_exposure_") + std::to_string(streamIdx), isAutomatic);
    setParameter(parameter);
}

void CameraControlWidget::preciseExposureTimeSetting(uint32_t streamIdx) {
    int value = m_lineEditExpoTime[streamIdx]->text().toInt();
    setExposureTime(value, streamIdx);
}

void CameraControlWidget::setProcParameter(uint32_t streamIdx) {
    if (!m_lineEditParams[streamIdx]->text().isEmpty()) {
        std_msgs::msg::String msg;
        msg.data = m_lineEditParams[streamIdx]->text().toStdString();
        m_lineEditParams[streamIdx]->clear();
        m_pubParameters[streamIdx]->publish(msg);
    }
}

} // namespace pmd_royale_ros_examples
