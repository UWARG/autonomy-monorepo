import { Bar, BarChart, Cell, Line, LineChart, Pie, PieChart, ResponsiveContainer, XAxis, YAxis } from 'recharts';
import type { CustomWidgetDefinition } from '../layout';

const SAMPLE = [{ name: '1', value: 18 }, { name: '2', value: 32 }, { name: '3', value: 24 }, { name: '4', value: 41 }];
const COLORS = ['#5B8DEF', '#34D17E', '#E0A83A', '#F26555'];

export default function CustomWidget({ definition }: { definition: CustomWidgetDefinition }) {
  const value = definition.decimals == null ? '42' : Number(42).toFixed(definition.decimals);
  return (
    <section className="widget flex h-full min-h-[120px] flex-col overflow-hidden p-4">
      <header className="flex items-center justify-between">
        <h2 className="widget-label">{definition.title}</h2>
        <span className="pill-accent">{definition.dataKind}</span>
      </header>
      <div className="mt-3 min-h-0 flex-1">
        {definition.renderer === 'number' && <div className="flex h-full items-center justify-center font-mono text-4xl font-semibold text-ink">{value}</div>}
        {definition.renderer === 'label' && <p className="text-[18px] text-ink-2">{definition.label || definition.field}</p>}
        {definition.renderer === 'button' && <button type="button" className="ims-button ims-button-primary mt-4">{definition.label || definition.title}</button>}
        {definition.renderer === 'coordinates' && <div className="font-mono text-sm text-ink-2">LAT 43.6532<br />LON -79.3832</div>}
        {['line', 'bar', 'pie'].includes(definition.renderer) && (
          <ResponsiveContainer width="100%" height="100%">
            {definition.renderer === 'line' ? (
              <LineChart data={SAMPLE}><XAxis dataKey="name" hide /><YAxis hide /><Line type="monotone" dataKey="value" stroke="#5B8DEF" strokeWidth={3} dot={false} /></LineChart>
            ) : definition.renderer === 'bar' ? (
              <BarChart data={SAMPLE}><XAxis dataKey="name" hide /><YAxis hide /><Bar dataKey="value" fill="#5B8DEF" radius={[5, 5, 0, 0]} /></BarChart>
            ) : (
              <PieChart><Pie data={SAMPLE} dataKey="value" nameKey="name" innerRadius="55%" outerRadius="80%" paddingAngle={3}>{SAMPLE.map((entry, index) => <Cell key={entry.name} fill={COLORS[index]} />)}</Pie></PieChart>
            )}
          </ResponsiveContainer>
        )}
      </div>
    </section>
  );
}
