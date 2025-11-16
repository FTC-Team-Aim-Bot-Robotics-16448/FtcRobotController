package org.firstinspires.ftc.teamcode.aim.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Menu {
    private MenuNode root;
    private MenuNode currentNode;
    private int selectedIndex;

    private Button upButton;
    private Button downButton;
    private Button selectButton;
    private Button backButton;

    public Menu() {
        root = new MenuNode("");
        currentNode = root;
        selectedIndex = 0;

        upButton = new Button();
        downButton = new Button();
        selectButton = new Button();
        backButton = new Button();
    }

    public void addMenuItem(String str) {
        if (str == null || str.isEmpty()) {
            return;
        }

        String[] parts = str.split("/");
        MenuNode current = root;

        for (String part : parts) {
            if (part.isEmpty()) {
                continue;
            }

            MenuNode child = current.getChild(part);
            if (child == null) {
                child = new MenuNode(part);
                current.addChild(child);
            }
            current = child;
        }
    }

    public String show(Gamepad gamepad, Telemetry telemetry) {
        String selectedOption = null;

        while (selectedOption == null) {
            upButton.update(gamepad.dpad_up);
            downButton.update(gamepad.dpad_down);
            selectButton.update(gamepad.right_bumper);
            backButton.update(gamepad.left_bumper);

            List<MenuNode> children = currentNode.getChildren();

            if (children.isEmpty()) {
                break;
            }

            if (upButton.isPressed()) {
                selectedIndex--;
                if (selectedIndex < 0) {
                    selectedIndex = children.size() - 1;
                }
            }

            if (downButton.isPressed()) {
                selectedIndex++;
                if (selectedIndex >= children.size()) {
                    selectedIndex = 0;
                }
            }

            if (backButton.isPressed() && currentNode != root) {
                currentNode = currentNode.getParent();
                selectedIndex = 0;
            }

            if (selectButton.isPressed()) {
                MenuNode selected = children.get(selectedIndex);

                if (selected.isLeaf()) {
                    selectedOption = getFullPath(selected);
                    currentNode = root;
                    selectedIndex = 0;
                } else {
                    currentNode = selected;
                    selectedIndex = 0;
                }
            }

            displayMenu(telemetry);
        }

        return selectedOption;
    }

    private void displayMenu(Telemetry telemetry) {
        telemetry.addData("", "D-Pad Up/Down: Navigate");
        telemetry.addData("", "Right Bumper: Select");
        if (currentNode != root) {
            telemetry.addData("", "Left Bumper: Back");
        }
        telemetry.addLine("---");

        List<MenuNode> children = currentNode.getChildren();
        for (int i = 0; i < children.size(); i++) {
            MenuNode child = children.get(i);
            String marker = (i == selectedIndex) ? "> " : "  ";
            String suffix = child.isLeaf() ? "" : " >";
            String displayName = getDisplayName(child.getName());
            telemetry.addLine(marker + displayName + suffix);
        }

        telemetry.addLine("---");
        telemetry.addData("Current Menu", getCurrentPath());

        telemetry.update();
    }

    private String getDisplayName(String name) {
        int hashIndex = name.indexOf('#');
        if (hashIndex != -1) {
            return name.substring(0, hashIndex);
        }
        return name;
    }

    private String getCurrentPath() {
        if (currentNode == root) {
            return "Main Menu";
        }
        return getFullPath(currentNode);
    }

    private String getFullPath(MenuNode node) {
        if (node == root || node.getParent() == null) {
            return "";
        }

        List<String> path = new ArrayList<>();
        MenuNode current = node;

        while (current != root && current != null) {
            path.add(0, current.getName());
            current = current.getParent();
        }

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < path.size(); i++) {
            sb.append(path.get(i));
            if (i < path.size() - 1) {
                sb.append("/");
            }
        }

        return sb.toString();
    }

    public void reset() {
        currentNode = root;
        selectedIndex = 0;
    }

    private static class MenuNode {
        private String name;
        private MenuNode parent;
        private Map<String, MenuNode> children;

        public MenuNode(String name) {
            this.name = name;
            this.children = new HashMap<>();
        }

        public String getName() {
            return name;
        }

        public MenuNode getParent() {
            return parent;
        }

        public void setParent(MenuNode parent) {
            this.parent = parent;
        }

        public void addChild(MenuNode child) {
            children.put(child.getName(), child);
            child.setParent(this);
        }

        public MenuNode getChild(String name) {
            return children.get(name);
        }

        public List<MenuNode> getChildren() {
            return new ArrayList<>(children.values());
        }

        public boolean isLeaf() {
            return children.isEmpty();
        }
    }
}
