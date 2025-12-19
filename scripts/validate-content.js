const fs = require('fs');
const path = require('path');
const matter = require('gray-matter');

const CONTENT_DIR = path.join(__dirname, '../content/modules');
const REQUIRED_SECTIONS = [
  'Learning Objectives',
  'Concept Explanations',
  'Visual Descriptions', // Or Conceptual Visualization
  'Hands-on Exercises',
  'Capstone Prep'
];

function validateFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const { data, content: markdownBody } = matter(content);
  const errors = [];

  // 1. Check Frontmatter
  if (!data.id) errors.push('Missing "id" in frontmatter');
  if (!data.title) errors.push('Missing "title" in frontmatter');
  if (!data.description) errors.push('Missing "description" in frontmatter');

  // 2. Check Required Sections
  REQUIRED_SECTIONS.forEach(section => {
    let found = false;
    
    // Check for "Visual Descriptions" OR "Conceptual Visualization"
    if (section === 'Visual Descriptions') {
         if (markdownBody.match(/^##\s+.*Visual.*/m) || 
             markdownBody.match(/^##\s+.*Diagrams.*/m) ||
             markdownBody.match(/^##\s+.*Conceptual Visualization.*/m)) {
             found = true;
         }
    } 
    // Check for "Hands-on" OR "Thought" exercises
    else if (section === 'Hands-on Exercises') {
        if (markdownBody.match(/^##\s+.*Hands-on.*/m) || 
            markdownBody.match(/^##\s+.*Thought Exercises.*/m)) {
            found = true;
        }
    } 
    // Default check
    else {
        if (markdownBody.match(new RegExp(`^##\\s+${section}`, 'm'))) found = true;
    }

    if (!found) errors.push(`Missing required section: "## ${section}" (or accepted alternative)`);
  });

  // 3. Check NO Images (Strict Mode)
  // Fail on ANY ![]() or <img /> tag
  const imgRegex = /!\[.*?]\[.*?]|<img.*?>/g;
  if (imgRegex.test(markdownBody)) {
      errors.push('Image detected! Images are STRICTLY FORBIDDEN in this text-only expansion phase.');
  }

  return errors;
}

function walkDir(dir) {
  let hasErrors = false;
  const files = fs.readdirSync(dir);
  files.forEach(file => {
    const fullPath = path.join(dir, file);
    if (fs.statSync(fullPath).isDirectory()) {
      if (walkDir(fullPath)) hasErrors = true;
    } else if (file.endsWith('.md') || file.endsWith('.mdx')) {
      if (file === 'index.md') return; 
      const errors = validateFile(fullPath);
      if (errors.length > 0) {
        console.error(`\n❌ Validation Failed: ${fullPath}`);
        errors.forEach(e => console.error(`   - ${e}`));
        hasErrors = true;
      }
    }
  });
  return hasErrors;
}

console.log('🔍 Validating content structure (Text-Only Mode)...');
const failed = walkDir(CONTENT_DIR);

if (failed) {
  console.error('\n💥 Content validation failed. Please fix the errors above.');
  process.exit(1);
} else {
  console.log('\n✅ Content validation passed!');
}