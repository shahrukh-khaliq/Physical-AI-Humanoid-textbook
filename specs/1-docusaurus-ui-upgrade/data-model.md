# Data Model: Docusaurus UI Upgrade for Book Frontend

## Phase 1: Design & Contracts

### Entity: Book Sections
**Fields**:
- id: unique identifier for the section
- title: display title of the section
- hierarchy: parent-child relationships in the navigation
- content_path: path to the actual content file
- metadata: additional information (author, date, etc.)

**Validation rules**:
- Must have a unique title within its parent section
- Hierarchy relationships must form a valid tree structure
- Content path must exist and be accessible

**Relationships**:
- Parent section (optional)
- Child sections (multiple)
- Navigation node (one-to-one)

### Entity: Navigation Elements
**Fields**:
- id: unique identifier
- label: display text for the navigation element
- link: URL or path to the target
- type: category, link, or doc
- position: order in the navigation hierarchy
- children: sub-elements (for dropdowns/menus)

**Validation rules**:
- Label must not be empty
- Link must be valid (internal or external)
- Position must be unique among siblings

**Relationships**:
- Parent navigation element (optional)
- Child navigation elements (multiple)
- Associated book section (optional)

### Entity: Visual Theme
**Fields**:
- id: unique identifier
- name: theme name (e.g., "light", "dark", "book-theme")
- colors: color palette (primary, secondary, background, text)
- typography: font family, sizes, weights
- spacing: margin, padding scale
- breakpoints: responsive design breakpoints

**Validation rules**:
- Color values must be valid CSS colors
- Typography values must be valid CSS font properties
- Breakpoints must be in ascending order

**Relationships**:
- Applied to: multiple pages/sections

### Entity: Page Layout
**Fields**:
- id: unique identifier
- type: page type (home, doc, blog, etc.)
- components: list of UI components in the layout
- configuration: layout-specific settings
- responsive_settings: mobile/tablet/desktop variations

**Validation rules**:
- Components must be valid Docusaurus or custom components
- Configuration must match component requirements

**State transitions**:
- Default layout → Custom layout (when UI changes are applied)

## UI Component Specifications

### Theme Configuration
- **Primary Color**: Brand color for highlights and links
- **Secondary Color**: Supporting color for backgrounds and accents
- **Typography Scale**: Responsive font sizes for headings and body text
- **Spacing Scale**: Consistent spacing system based on design tokens

### Navigation Components
- **Sidebar**: Hierarchical navigation with collapsible sections
- **Navbar**: Top navigation with logo, search, and utility links
- **Breadcrumbs**: Path visualization for current location
- **Table of Contents**: In-page navigation for long documents

### Content Components
- **Typography**: Consistent heading hierarchy and body text styling
- **Code Blocks**: Syntax highlighting with copy functionality
- **Callout Blocks**: Info, warning, and note styling
- **Media Components**: Responsive image and video handling