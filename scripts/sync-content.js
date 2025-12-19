const fs = require('fs-extra');
const path = require('path');
const chokidar = require('chokidar');

const SOURCE_DIR = path.join(__dirname, '../content/modules');
const TARGET_DIR = path.join(__dirname, '../apps/web/docs');

async function sync() {
  try {
    // Ensure target exists
    await fs.ensureDir(TARGET_DIR);

    // Empty target first to remove deleted files (optional, risky if mixing content)
    // await fs.emptyDir(TARGET_DIR); 

    // Copy contents
    await fs.copy(SOURCE_DIR, TARGET_DIR, {
      overwrite: true,
      errorOnExist: false
    });
    console.log(`✅ Synced content from ${SOURCE_DIR} to ${TARGET_DIR}`);
  } catch (err) {
    console.error('❌ Sync failed:', err);
    process.exit(1);
  }
}

// Initial sync
sync();

// Watch mode if arg provided
if (process.argv.includes('--watch')) {
  console.log('👀 Watching for changes...');
  chokidar.watch(SOURCE_DIR, { ignoreInitial: true }).on('all', (event, path) => {
    console.log(`File ${event}: ${path}`);
    sync();
  });
}
