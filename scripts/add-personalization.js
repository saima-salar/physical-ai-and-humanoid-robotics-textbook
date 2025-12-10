const fs = require('fs');
const path = require('path');

// Function to add personalization button to a markdown file
function addPersonalizationButton(filePath) {
  let content = fs.readFileSync(filePath, 'utf8');

  // Check if personalization button is already added
  if (content.includes('PersonalizationButton')) {
    console.log(`✅ Already added to: ${filePath}`);
    return;
  }

  // Extract the frontmatter and title
  const frontmatterMatch = content.match(/---\n([\s\S]*?)\n---/);

  let id, title;

  if (frontmatterMatch) {
    // If frontmatter exists, extract ID and title from it
    const frontmatter = frontmatterMatch[1];
    const idMatch = frontmatter.match(/id: (.+)/);
    const titleMatch = frontmatter.match(/title: (.+)/);

    id = idMatch ? idMatch[1].trim() : path.basename(filePath, '.md');
    title = titleMatch ? titleMatch[1].replace(/['"]/g, '') : 'Untitled Chapter';

    // Create the import statements and component insertion
    const importStatements = `import PersonalizationButton from '@site/src/components/PersonalizationButton';
import ContentFilter from '@site/src/components/ContentFilter';`;

    const componentInsertion = `<PersonalizationButton chapterId="${id}" chapterTitle="${title}" />`;

    // Replace the frontmatter with frontmatter + imports + component
    const updatedContent = content.replace(
      frontmatterMatch[0],
      `${frontmatterMatch[0]}\n\n${importStatements}\n\n${componentInsertion}`
    );

    fs.writeFileSync(filePath, updatedContent);
    console.log(`✅ Added to: ${filePath} (with frontmatter)`);
  } else {
    // If no frontmatter exists, extract title from the first H1 and use filename as ID
    const h1Match = content.match(/^# (.+)$/m);
    title = h1Match ? h1Match[1].trim() : 'Untitled Chapter';
    id = path.basename(filePath, '.md');

    // Create the import statements and component insertion
    const importStatements = `import PersonalizationButton from '@site/src/components/PersonalizationButton';
import ContentFilter from '@site/src/components/ContentFilter';`;

    const componentInsertion = `<PersonalizationButton chapterId="${id}" chapterTitle="${title}" />\n\n`;

    // Insert at the beginning of the file
    const updatedContent = importStatements + '\n\n' + componentInsertion + content;

    fs.writeFileSync(filePath, updatedContent);
    console.log(`✅ Added to: ${filePath} (without frontmatter)`);
  }
}

// Get all markdown files in docs directory
function processDocsDirectory(docsPath) {
  const files = fs.readdirSync(docsPath);

  files.forEach(file => {
    if (file.endsWith('.md') && file.startsWith('chapter-')) {
      const filePath = path.join(docsPath, file);
      addPersonalizationButton(filePath);
    }
  });
}

// Run the script
const docsDirectory = './docs';
processDocsDirectory(docsDirectory);

console.log('🎉 Personalization buttons added to all chapter files!');