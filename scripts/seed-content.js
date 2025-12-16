const fs = require('fs');
const path = require('path');

const modules = [
  {
    id: '01-intro',
    title: 'Introduction to Physical AI & Embodied Intelligence',
    sidebar_position: 1,
  },
  { id: '02-ros2', title: 'ROS 2 Fundamentals', sidebar_position: 2 },
  { id: '03-gazebo-unity', title: 'Gazebo & Unity Simulation', sidebar_position: 3 },
  { id: '04-nvidia-isaac', title: 'NVIDIA Isaac Platform', sidebar_position: 4 },
  { id: '05-vla', title: 'Vision-Language-Action (VLA)', sidebar_position: 5 },
  { id: '06-humanoid', title: 'Humanoid Robotics & Locomotion', sidebar_position: 6 },
  { id: '07-conversational', title: 'Conversational Robotics', sidebar_position: 7 },
  { id: '08-capstone', title: 'Capstone Project Guidance', sidebar_position: 8 },
];

const contentDir = path.join(__dirname, '../content/modules');

if (!fs.existsSync(contentDir)) {
  fs.mkdirSync(contentDir, { recursive: true });
}

modules.forEach((mod) => {
  const filePath = path.join(contentDir, `${mod.id}.md`);
  const content = `---
id: ${mod.id}
title: ${mod.title}
sidebar_position: ${mod.sidebar_position}
description: ${mod.title} module for the AI-Native Textbook.
---

# ${mod.title}

*Status: Under Construction*

This is a placeholder for the **${mod.title}** module.
`;

  if (!fs.existsSync(filePath)) {
    fs.writeFileSync(filePath, content);
    console.log(`Created: ${mod.id}.md`);
  } else {
    console.log(`Skipped (exists): ${mod.id}.md`);
  }
});

console.log('Content seeding complete.');
