#include "playertia/esp32_s3/app_shell.hpp"

namespace playertia::esp32_s3 {

bool Application::handleButton(AppContext&,
                               const HardwareButton,
                               const ButtonAction) noexcept {
  return false;
}

AppManager::AppManager(AppContext& context) noexcept : context_(context) {}

bool AppManager::registerApplication(Application& application) noexcept {
  if (application_count_ >= applications_.size()) {
    return false;
  }

  for (std::size_t index = 0; index < application_count_; ++index) {
    if (applications_[index] == &application ||
        applications_[index]->id() == application.id()) {
      return false;
    }
  }

  applications_[application_count_] = &application;
  ++application_count_;
  return true;
}

bool AppManager::start(const std::size_t initial_index) noexcept {
  if (active_index_ != no_active_application) {
    return true;
  }
  if (initial_index >= application_count_) {
    return false;
  }
  if (!applications_[initial_index]->load(context_)) {
    return false;
  }

  active_index_ = initial_index;
  return true;
}

void AppManager::stop() noexcept {
  if (active_index_ == no_active_application) {
    return;
  }

  applications_[active_index_]->unload(context_);
  active_index_ = no_active_application;
}

SwitchResult AppManager::handleButton(const HardwareButton button,
                                      const ButtonAction action) noexcept {
  if (button != HardwareButton::top_right) {
    return SwitchResult::ignored;
  }
  if (action == ButtonAction::long_pressed) {
    return switchToNext();
  }
  if (active_index_ != no_active_application &&
      applications_[active_index_]->handleButton(context_, button, action)) {
    return SwitchResult::application_handled;
  }
  return SwitchResult::ignored;
}

SwitchResult AppManager::switchToNext() noexcept {
  if (application_count_ == 0) {
    return SwitchResult::no_applications;
  }
  if (application_count_ == 1) {
    return start(0) ? SwitchResult::single_application
                    : SwitchResult::load_failed_inactive;
  }
  if (active_index_ == no_active_application) {
    return start(0) ? SwitchResult::switched
                    : SwitchResult::load_failed_inactive;
  }

  const std::size_t previous_index = active_index_;
  const std::size_t next_index = (active_index_ + 1) % application_count_;
  applications_[previous_index]->unload(context_);
  active_index_ = no_active_application;

  if (applications_[next_index]->load(context_)) {
    active_index_ = next_index;
    return SwitchResult::switched;
  }

  if (applications_[previous_index]->load(context_)) {
    active_index_ = previous_index;
    return SwitchResult::load_failed_restored;
  }
  return SwitchResult::load_failed_inactive;
}

void AppManager::update(const std::uint32_t elapsed_ms) noexcept {
  if (active_index_ != no_active_application) {
    applications_[active_index_]->update(context_, elapsed_ms);
  }
}

void AppManager::render() noexcept {
  if (active_index_ != no_active_application) {
    applications_[active_index_]->render(context_);
  }
}

Application* AppManager::activeApplication() noexcept {
  return active_index_ == no_active_application
             ? nullptr
             : applications_[active_index_];
}

const Application* AppManager::activeApplication() const noexcept {
  return active_index_ == no_active_application
             ? nullptr
             : applications_[active_index_];
}

std::size_t AppManager::activeIndex() const noexcept {
  return active_index_;
}

std::size_t AppManager::applicationCount() const noexcept {
  return application_count_;
}

}  // namespace playertia::esp32_s3
