# Quickstart: UI/UX Polish Verification

## Development Setup

1. **Install Dependencies**:
   ```bash
   npm install framer-motion
   ```

2. **Run Local Server**:
   ```bash
   cd apps/web
   npm run start
   ```

## Verification Checklist

### 1. Visual Inspection (Landing Page)
- [ ] **Navbar**: Scroll down; background should blur content behind it (Glassmorphism).
- [ ] **Hero Section**: Verify presence of animating "glow blobs" in the background.
- [ ] **Cards**: Hover over cards; they should scale up slightly and gain a neon cyan outer glow.

### 2. Reading Experience
- [ ] **Scrollbar**: Verify slim, cyan scrollbar on long pages.
- [ ] **Code Blocks**:
  - [ ] macOS-style red/yellow/green buttons in header.
  - [ ] Language label on the right side of header.
  - [ ] Copy button integrated into header.

### 3. Performance
- [ ] Open Chrome DevTools > Performance.
- [ ] Scroll through the landing page.
- [ ] Verify "Frames" stay at ~60fps without red bars (jank).

## Troubleshooting
- **Blur not working**: Ensure `backdrop-filter` is not disabled in browser settings and check for `-webkit-` prefix in CSS.
- **Animations laggy**: Ensure only `transform` and `opacity` are being animated.