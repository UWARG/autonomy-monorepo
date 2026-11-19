
export default {
  content: ['./index.html', './src/**/*.{ts,tsx,js,jsx}'],
  theme: {
    extend: {
      colors: {
        // surfaces
        app: 'rgb(var(--ims-app) / <alpha-value>)',      // page background
        card: 'rgb(var(--ims-card) / <alpha-value>)',     // widget surface
        edge: 'rgb(var(--ims-edge) / <alpha-value>)',     // control borders
        inset: 'rgb(var(--ims-inset) / <alpha-value>)',   // recessed areas (map bg)
        feed: 'rgb(var(--ims-feed) / <alpha-value>)',     // camera feed background

        // text
        ink: {
          DEFAULT: 'rgb(var(--ims-ink) / <alpha-value>)', // primary
          2: 'rgb(var(--ims-ink-2) / <alpha-value>)',     // secondary
          3: 'rgb(var(--ims-ink-3) / <alpha-value>)',     // labels / captions
        },

        // semantic state
        ok: { DEFAULT: '#218739', dim: '#E5F4E8' },
        warn: { DEFAULT: '#A86600', dim: '#FFF3D6' },
        bad: { DEFAULT: '#C62828', dim: '#FDEAEA' },
        accent: {
          DEFAULT: 'rgb(var(--ims-accent) / <alpha-value>)',
          dim: 'rgb(var(--ims-accent-dim) / <alpha-value>)',
        },

        // artificial horizon
        sky: '#BFE0F5',
        ground: '#6E5744',
      },
      fontFamily: {
        sans: ['Roboto', 'system-ui', 'sans-serif'],
        mono: ['JetBrains Mono', 'ui-monospace', 'monospace'],
      },
      borderRadius: {
        widget: '14px',
      },
    },
  },
  plugins: [],
};