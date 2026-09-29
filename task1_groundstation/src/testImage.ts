/** A synthetic placeholder image, drawn so it's obviously not a real capture. */
export function makeTestImageDataUrl(label: string): string {
  const width = 640;
  const height = 480;
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext('2d');
  if (!ctx) return '';

  ctx.fillStyle = '#d4d4d8';
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = '#71717a';
  ctx.lineWidth = 4;
  ctx.strokeRect(2, 2, width - 4, height - 4);

  ctx.fillStyle = '#3f3f46';
  ctx.textAlign = 'center';
  ctx.font = 'bold 28px sans-serif';
  ctx.fillText('TEST IMAGE', width / 2, height / 2 - 20);
  ctx.font = '16px sans-serif';
  ctx.fillText(label, width / 2, height / 2 + 10);
  ctx.fillText(new Date().toLocaleTimeString(), width / 2, height / 2 + 34);

  return canvas.toDataURL('image/png');
}
