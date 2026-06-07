#include "mode_display_node.h"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

namespace
{
constexpr float kTargetFrameTimeSec = 1.0F / 60.0F;
constexpr float kMainWindowPosX = 20.0F;
constexpr float kMainWindowPosY = 20.0F;
constexpr float kMainWindowWidth = 860.0F;
constexpr float kMainWindowHeight = 500.0F;
constexpr float kModeFontScale = 2.8F;
const ImVec4 kInteractiveColor = ImVec4(1.0F, 0.55F, 0.0F, 1.0F);
const ImVec4 kAutonomousColor = ImVec4(0.0F, 0.8F, 0.2F, 1.0F);
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ModeDisplayNode>();

  if (!glfwInit()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize GLFW");
    rclcpp::shutdown();
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  GLFWwindow * window = glfwCreateWindow(900, 540, "ROS 2 Navigation Mode Display", nullptr, nullptr);
  if (window == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create GLFW window");
    glfwTerminate();
    rclcpp::shutdown();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");

  while (!glfwWindowShouldClose(window) && rclcpp::ok()) {
    const auto frame_start = std::chrono::steady_clock::now();

    glfwPollEvents();
    rclcpp::spin_some(node);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(kMainWindowPosX, kMainWindowPosY), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(kMainWindowWidth, kMainWindowHeight), ImGuiCond_Once);
    ImGui::Begin("ROS 2 Navigation Mode Display");

    const bool interactive_mode = node->is_interactive_mode();
    const ImVec4 mode_color = interactive_mode ? kInteractiveColor : kAutonomousColor;
    const char * mode_text = interactive_mode ? "Interactive Mode" : "Autonomous Mode";

    ImGui::Spacing();
    ImGui::Text("Navigation Mode");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::SetWindowFontScale(kModeFontScale);
    const ImVec2 text_size = ImGui::CalcTextSize(mode_text);
    const float window_width = ImGui::GetContentRegionAvail().x;
    ImGui::SetCursorPosX((window_width - text_size.x) * 0.5F);
    ImGui::TextColored(mode_color, "%s", mode_text);
    ImGui::SetWindowFontScale(1.0F);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    const auto message_count = node->message_count();
    const auto last_message_time = node->last_message_time();
    const bool has_received = node->has_received_message();
    const auto publisher_count = node->count_publishers(node->topic_name());

    ImGui::Text("Topic: %s", node->topic_name().c_str());
    ImGui::Text("Publisher count: %zu", publisher_count);
    ImGui::Text("Message count: %zu", message_count);

    if (has_received) {
      const double stamp_sec = last_message_time.seconds();
      ImGui::Text("Last update: %.3f sec", stamp_sec);
    } else {
      ImGui::Text("Last update: waiting for first message...");
    }

    ImGui::End();

    ImGui::Render();
    int display_w = 0;
    int display_h = 0;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1F, 0.1F, 0.1F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);

    const auto frame_end = std::chrono::steady_clock::now();
    const std::chrono::duration<float> elapsed = frame_end - frame_start;
    if (elapsed.count() < kTargetFrameTimeSec) {
      std::this_thread::sleep_for(
        std::chrono::duration<float>(kTargetFrameTimeSec - elapsed.count()));
    }
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  rclcpp::shutdown();

  return 0;
}
