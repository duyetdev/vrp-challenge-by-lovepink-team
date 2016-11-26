FROM prodataninja/ubuntu-python2.7

#use gunicorn
RUN pip install gunicorn==19.6.0

#Install requirements
COPY requirements.txt /usr/src/app/
WORKDIR /usr/src/app
RUN pip install -r requirements.txt
RUN pip install gunicorn

COPY . /usr/src/app/

RUN set -x \
    && apt-get update \
    && pip install --upgrade pip \
    && curl https://bootstrap.pypa.io/ez_setup.py -o - | python \
    && pip install ez_setup \
    && wget https://github.com/google/or-tools/releases/download/v2016-06/Google.OrTools.python.examples.3631.tar.gz \
    && tar -zxf Google.OrTools.python.examples.3631.tar.gz \
    && cd ortools_examples \
    && python setup.py install \
    && pip install --upgrade ortools

RUN set -x \
    && python ortools_examples/examples/python/golomb8.py

EXPOSE 5000
ENTRYPOINT ["/usr/local/bin/gunicorn"]
#CMD -w 3 -b 0.0.0.0:5000 --timeout=120 --reload --access-logfile /var/log/access.log server:app
CMD ["main:api", "-b", "0.0.0.0:5000"]
