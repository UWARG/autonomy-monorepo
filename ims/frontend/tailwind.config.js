
export default {
  content: ['./index.html', './src/**/*.{ts,tsx,js,jsx}'],
  theme: {
    extend: {
      colors: {
        // surfaces
        app: '#0C0F16',      // page background
        card: '#151A24',     // widget surface
        edge: '#252C3A',     // hairline borders / dividers
        inset: '#0E131C',    // recessed areas (map bg)
        feed: '#080B11',     // camera feed background

        // text
        ink: {
          DEFAULT: '#EAEEF5', // primary
          2: '#9BA5B7',       // secondary
          3: '#5E687A',       // labels / captions
        },

        // semantic state
        ok: { DEFAULT: '#34D17E', dim: '#12331F' },
        warn: { DEFAULT: '#E0A83A', dim: '#3A2E12' },
        bad: { DEFAULT: '#F26555', dim: '#3A1A16' },
        accent: { DEFAULT: '#5B8DEF', dim: '#16233F' },

        // artificial horizon
        sky: '#BFE0F5',
        ground: '#6E5744',
      },
      fontFamily: {
        sans: ['Inter', 'system-ui', 'sans-serif'],
        mono: ['JetBrains Mono', 'ui-monospace', 'monospace'],
      },
      borderRadius: {
        widget: '14px',
      },
    },
  },
  plugins: [],
};