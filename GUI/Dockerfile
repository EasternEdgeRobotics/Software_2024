FROM node:alpine as build-stage
COPY . /root/GUI
WORKDIR /root/GUI
RUN npm install
RUN npm run build
FROM nginx:alpine
COPY nginx.conf /etc/nginx/nginx.conf
COPY --from=build-stage /root/GUI/build /usr/share/nginx/html
RUN rm /etc/nginx/conf.d/default.conf  # <= This line solved my issue