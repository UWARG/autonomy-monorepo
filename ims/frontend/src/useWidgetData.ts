import { useState, useEffect } from 'react'
import { subscribe, unsubscribe } from './socket'

export function useWidgetData(topics: string[]): Record<string, unknown> {
  const [data, setData] = useState<Record<string, unknown>>({})

  useEffect(() => {
    // topics is derived from a static widget definition — safe to ignore exhaustive-deps
    const handlers = topics.map(topic => {
      const handler = (payload: unknown) => {
        setData(prev => ({ ...prev, [topic]: payload }))
      }
      subscribe(topic, handler)
      return { topic, handler }
    })

    return () => {
      handlers.forEach(({ topic, handler }) => unsubscribe(topic, handler))
    }
  }, []) // eslint-disable-line react-hooks/exhaustive-deps

  return data
}
