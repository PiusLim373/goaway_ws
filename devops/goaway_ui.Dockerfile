FROM node:22-slim AS build

WORKDIR /app
COPY app/goaway_ui/package.json app/goaway_ui/package-lock.json ./
RUN npm ci
COPY app/goaway_ui/ ./
RUN npm run build

FROM node:22-slim

ENV NODE_ENV=production
WORKDIR /app

COPY app/goaway_ui/package.json app/goaway_ui/package-lock.json ./
RUN npm ci --omit=dev

COPY --from=build /app/dist ./dist
COPY app/goaway_ui/ui_backend.js ./

RUN chown -R node:node /app
USER node

EXPOSE 3001
CMD ["node", "ui_backend.js"]
