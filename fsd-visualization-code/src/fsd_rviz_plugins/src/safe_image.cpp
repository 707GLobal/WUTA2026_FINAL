// SPDX-License-Identifier: Apache-2.0
#include <message_filters/connection.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_default_plugins/displays/image/image_display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <QDockWidget>
#include <QMainWindow>
#include <QScreen>
#include <QTimer>

namespace fsd_rviz_plugins
{
class SafeImage : public rviz_default_plugins::displays::ImageDisplay
{
public:
  ~SafeImage() override {unsubscribe();}

protected:
  void unsubscribe() override
  {
    // Clear the connection before releasing the filter; base teardown may
    // disconnect again. Keep normal Image rendering and transport behavior.
    if (subscription_) {
      subscription_callback_.disconnect();
    }
    subscription_callback_ = message_filters::Connection();
    subscription_.reset();
  }

  void onDisable() override
  {
    unsubscribe();
    rviz_default_plugins::displays::ImageDisplay::onDisable();
  }
};

class DashboardImage : public SafeImage
{
public:
  void onInitialize() override
  {
    SafeImage::onInitialize();
    // Wait until RViz has finished loading displays and window geometry.
    QTimer::singleShot(0, this, [this]() {
      auto * window = qobject_cast<QMainWindow *>(
        context_->getWindowManager()->getParentWindow());
      if (!window) {return;}
      QDockWidget * camera = nullptr;
      QDockWidget * displays = nullptr;
      QDockWidget * views = nullptr;
      for (auto * dock : window->findChildren<QDockWidget *>()) {
        if (dock->windowTitle() == getName()) {camera = dock;}
        if (dock->windowTitle() == "Displays") {displays = dock;}
        if (dock->windowTitle() == "Views") {views = dock;}
      }
      if (!camera || !displays || !views) {return;}
      for (auto * dock : {camera, displays, views}) {
        dock->setFloating(false);
        window->addDockWidget(Qt::LeftDockWidgetArea, dock);
      }
      // Match the reference: camera, display properties, then view properties.
      window->splitDockWidget(camera, displays, Qt::Vertical);
      window->splitDockWidget(displays, views, Qt::Vertical);
      for (auto * dock : {camera, displays, views}) {dock->show();}
      window->showMaximized();
      QTimer::singleShot(0, camera, [window, camera, displays, views]() {
        const auto size = window->screen()->availableGeometry().size();
        window->resizeDocks({camera, displays, views},
          {size.width() * 2 / 5, size.width() * 2 / 5, size.width() * 2 / 5},
          Qt::Horizontal);
        window->resizeDocks({camera, displays, views},
          {size.height() * 45 / 100, size.height() * 30 / 100,
            size.height() * 25 / 100}, Qt::Vertical);
      });
    });
  }
};
}  // namespace fsd_rviz_plugins
PLUGINLIB_EXPORT_CLASS(fsd_rviz_plugins::SafeImage, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(fsd_rviz_plugins::DashboardImage, rviz_common::Display)
