import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics Textbook',
  tagline: 'Exploring the Frontiers of Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-github-org', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics-textbook', // Usually your repo name.

  onBrokenLinks: 'throw',
  markdown: {
    mermaid: true,
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
    },
    // Math support is configured in the docs plugin below
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Even if you don't use internationalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace 'en' with 'zh-Hans'.
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo. Latest version: https://docusaurus.io/docs/next/docs-introduction#configure-docs-plugin
          // editUrl: 'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          remarkPlugins: [require('remark-math')],
          rehypePlugins: [require('rehype-katex')],
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo. Latest version: https://docusaurus.io/docs/next/blog-introduction#configure-blog-plugin
          // editUrl: 'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: [
            require.resolve('katex/dist/katex.min.css'),
            './src/css/custom.css',
            './src/css/design-system.css', // New design system CSS
          ],
        },
      }),
    ],
  ],

  // Environment variables for the chatbot and other components
  customFields: {
    BACKEND_API_URL: process.env.BACKEND_API_URL || 'http://localhost:8003',
  },

  // Google Fonts integration
  stylesheets: [
    'https://fonts.googleapis.com/css2?family=Inter:wght@100;200;300;400;500;600;700;800;900&display=swap',
    'https://fonts.googleapis.com/css2?family=Source+Code+Pro:wght@200;300;400;500;600;700;800;900&display=swap',
  ],

  scripts: [
    {
      src: '/js/env-injector.js',
      defer: true,
    },
    // {
    //   src: '/js/chatbot-injector.js',
    //   defer: true,
    // },
  ],
  plugins: [
    [
      './src/plugins/docusaurus-plugin-chatbot',
      {
        // Chatbot plugin options (if any)
      }
    ]
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI and Humanoid Robotics',
        logo: {
          alt: 'Physical AI and Humanoid Robotics Textbook Logo',
          src: '/img/book-logo.svg',
          height: 80,
          width: 80,
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            type: 'dropdown',
            label: 'Account',
            position: 'right',
            items: [
              {
                label: 'Sign In',
                to: '/signin',
              },
              {
                label: 'Sign Up',
                to: '/signup',
              },
            ],
          },
          {
            type: 'html',
            position: 'right',
            value: '<button id="global-chatbot-toggle" class="clean-btn navbar__link" onclick="openChatbotModal()" style="padding: 0; margin-left: 1rem; background: none; border: none; cursor: pointer; font-size: 1.2rem; position: relative;" title="AI Assistant">💬<span id="chatbot-indicator" style="position: absolute; top: -3px; right: -3px; width: 10px; height: 10px; background-color: #4caf50; border-radius: 50%; border: 1px solid white;"></span></button>',
          },
          // {
          //   to: '/blog',
          //   label: 'Blog',
          //   position: 'left'
          // },
          // {
          //   href: 'https://github.com/facebook/docusaurus',
          //   label: 'GitHub',
          //   position: 'right',
          // },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Textbook',
                to: '/docs/chapter-01-introduction-to-physical-ai',
              },
            ],
          },
          // {
          //   title: 'Community',
          //   items: [
          //     {
          //       label: 'Stack Overflow',
          //       href: 'https://stackoverflow.com/questions/tagged/docusaurus',
          //     },
          //     {
          //       label: 'Discord',
          //       href: 'https://discordapp.com/invite/docusaurus',
          //     },
          //     {
          //       label: 'Twitter',
          //       href: 'https://twitter.com/docusaurus',
          //     },
          //   ],
          // },
          // {
          //   title: 'More',
          //   items: [
          //     {
          //       label: 'Blog',
          //       to: '/blog',
          //     },
          //     {
          //       label: 'GitHub',
          //       href: 'https://github.com/facebook/docusaurus',
          //     },
          //   ],
          // },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI and Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
    }),
};

export default config;
