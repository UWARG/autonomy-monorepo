# IMS frontend

The IMS frontend is a React/Vite proof of concept for a modular ground-station
dashboard. Widgets are rendered from a versioned layout document instead of being
positioned directly in `App.tsx`. The editor uses React Grid Layout for
12-column snapping, dragging, and resizing.

## Run and validate

From `ims/frontend`:

```sh
npm install
npm run dev
npm run lint
npm run build
```

The dashboard expects rosbridge at the WebSocket URL derived in
`src/constants.ts`. Widgets that do not have live data show their existing
empty states, so the layout editor can be exercised without a connected
vehicle.

## Edit a layout

Use **Edit layout** in the toolbar to:

- select a widget and edit its `X`, `Y`, `W`, and `H` grid values;
- change the selected widget type from the editor bar to change what appears;
- drag from the bottom-right corner affordance;
- resize from the bottom-right corner;
- remove a widget with its top-right `×`;
- show the dotted snap grid;
- add any widget not already shown when nothing is selected;
- reset to the built-in default.

Each widget type has a minimum width and height so its internal padding and
controls remain usable. React Grid Layout also vertically compacts the layout
after a move or removal, allowing the remaining widgets to re-arrange.

Changes are saved automatically in the browser under the `ims-layout-v2`
local-storage key. **Export JSON** downloads the current layout. **Import JSON**
loads a layout after parsing and normalizing its known widget entries; malformed
files or files without known widgets are reported in the toolbar.

The editor uses a Material dashboard-inspired shell with a permanent navigation
rail, restrained bordered cards, and compact controls. The visual shell can be
restyled without changing the layout model or telemetry adapters.

## Custom widgets

While editing with no widget selected, choose **Create custom widget**. The POC
builder supports:

- raw numbers with configurable decimal places;
- line, bar, and pie charts;
- coordinates;
- labels;
- buttons.

Each custom widget chooses an IMS data kind first, then only fields compatible
with that kind are offered. Recharts renders the chart variants. The current
chart renderer uses sample values until the corresponding live telemetry
adapter is connected; the selected data kind and field are still persisted in
the custom widget definition.

Custom definitions are stored in the versioned layout JSON and are included in
export/import. Unsupported renderer/data combinations are rejected during
normalization. Arbitrary JavaScript expressions are intentionally not accepted
by this POC.

Existing widgets also expose display configuration where supported. For
example, selecting **Attitude**, entering edit mode, and opening **Elements**
allows the horizon graph and numeric Roll/Pitch/Yaw readouts to be toggled
independently.

## Add a widget

1. Create a React component in `src/widgets`.
2. Add its `WidgetType`, label, and default item to `src/layout.ts`.
3. Add a renderer entry in `src/WidgetRenderer.tsx`.
4. Run `npm run lint` and `npm run build`.

Keep layout items limited to configuration (`id`, type, position, and size).
ROS messages and other live telemetry should stay outside the saved JSON
document so layouts can be shared between different sessions.
