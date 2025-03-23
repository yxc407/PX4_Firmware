#!/usr/bin/env groovy

pipeline {
  agent none
  stages {

    stage('Analysis') {
      when {
        anyOf {
          branch 'main'
          branch 'master' // should be removed, but in case there is something going on...
          branch 'pr-jenkins' // for testing
        }
      }
      parallel {

        stage('Airframe') {
          agent {
            docker { image 'tcio/tc-dev-base-focal:2021-08-18' }
          }
          steps {
            sh 'make distclean; git clean -ff -x -d .'
            sh 'git fetch --all --tags'
            sh 'make airframe_metadata'
            dir('build/tc_sitl_default/docs') {
              archiveArtifacts(artifacts: 'airframes.md, airframes.xml')
              stash includes: 'airframes.md, airframes.xml', name: 'metadata_airframes'
            }
          }
          post {
            always {
              sh 'make distclean; git clean -ff -x -d .'
            }
          }
        }

        stage('Parameter') {
          agent {
            docker { image 'tcio/tc-dev-base-focal:2021-08-18' }
          }
          steps {
            sh 'make distclean; git clean -ff -x -d .'
            sh 'git fetch --all --tags'
            sh 'make parameters_metadata'
            dir('build/tc_sitl_default/docs') {
              archiveArtifacts(artifacts: 'parameters.md, parameters.xml, parameters.json.xz')
              stash includes: 'parameters.md, parameters.xml, parameters.json.xz', name: 'metadata_parameters'
            }
          }
          post {
            always {
              sh 'make distclean; git clean -ff -x -d .'
            }
          }
        }

        stage('Module') {
          agent {
            docker { image 'tcio/tc-dev-base-focal:2021-08-18' }
          }
          steps {
            sh 'make distclean; git clean -ff -x -d .'
            sh 'git fetch --all --tags'
            sh 'make module_documentation'
            dir('build/tc_sitl_default/docs') {
              archiveArtifacts(artifacts: 'modules/*.md')
              stash includes: 'modules/*.md', name: 'metadata_module_documentation'
            }
          }
          post {
            always {
              sh 'make distclean; git clean -ff -x -d .'
            }
          }
        }

        stage('msg file docs') {
          agent {
            docker { image 'tcio/tc-dev-base-focal:2021-08-18' }
          }
          steps {
            sh 'mkdir -p build/msg_docs; ./Tools/msg/generate_msg_docs.py -d build/msg_docs'
            dir('build') {
              archiveArtifacts(artifacts: 'msg_docs/*.md')
              stash includes: 'msg_docs/*.md', name: 'msg_documentation'
            }
          }
          post {
            always {
              sh 'make distclean; git clean -ff -x -d .'
            }
          }
        }

        stage('failsafe docs') {
          agent {
            docker { image 'tcio/tc-dev-nuttx-focal:2022-08-12' }
          }
          steps {
            sh '''#!/bin/bash -l
            echo $0;
            git clone https://github.com/emscripten-core/emsdk.git _emscripten_sdk;
            cd _emscripten_sdk;
            ./emsdk install latest;
            ./emsdk activate latest;
            cd ..;
            . ./_emscripten_sdk/emsdk_env.sh;
            make failsafe_web;
            cd build/tc_sitl_default_failsafe_web;
            mkdir -p failsafe_sim;
            cp index.* parameters.json failsafe_sim;
            '''
            dir('build/tc_sitl_default_failsafe_web') {
              archiveArtifacts(artifacts: 'failsafe_sim/*')
              stash includes: 'failsafe_sim/*', name: 'failsafe_sim'
            }
          }
          post {
            always {
              sh 'make distclean; git clean -ff -x -d .'
            }
          }
        }

        stage('uORB graphs') {
          agent {
            docker {
              image 'tcio/tc-dev-nuttx-focal:2022-08-12'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; git clean -ff -x -d .'
            sh 'git fetch --all --tags'
            sh 'make uorb_graphs'
            dir('Tools/uorb_graph') {
              archiveArtifacts(artifacts: 'graph_*.json')
              stash includes: 'graph_*.json', name: 'uorb_graph'
            }
          }
          post {
            always {
              sh 'make distclean; git clean -ff -x -d .'
            }
          }
        }

      } // parallel
    } // stage: Generate Metadata

    stage('Deploy') {

      parallel {

        stage('Userguide') {
          agent {
            docker { image 'tcio/tc-dev-base-focal:2021-08-18' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            unstash 'metadata_module_documentation'
            unstash 'msg_documentation'
            unstash 'failsafe_sim'
            unstash 'uorb_graph'
            withCredentials([usernamePassword(credentialsId: 'tcbuildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/TC/TC-user_guide.git')
              sh('cp airframes.md TC-user_guide/en/airframes/airframe_reference.md')
              sh('cp parameters.md TC-user_guide/en/advanced_config/parameter_reference.md')
              sh('cp -R modules/*.md TC-user_guide/en/modules/')
              sh('cp -R graph_*.json TC-user_guide/.vuepress/public/en/middleware/')
              sh('cp -R msg_docs/*.md TC-user_guide/en/msg_docs/')
              sh('cp -R failsafe_sim/* TC-user_guide/.vuepress/public/en/config/failsafe')
              sh('cd TC-user_guide; git status; git add .; git commit -a -m "Update TC Firmware metadata `date`" || true')
              sh('cd TC-user_guide; git push origin main || true')
              sh('rm -rf TC-user_guide')
            }
          }
          when {
            anyOf {
              branch 'main'
              branch 'master' // should be removed, but in case there is something going on...
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

        stage('QGroundControl') {
          agent {
            docker { image 'tcio/tc-dev-base-focal:2021-08-18' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            withCredentials([usernamePassword(credentialsId: 'tcbuildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/mavlink/qgroundcontrol.git')
              sh('cp airframes.xml qgroundcontrol/src/AutoPilotPlugins/TC/AirframeFactMetaData.xml')
              sh('cp parameters.xml qgroundcontrol/src/FirmwarePlugin/TC/TCParameterFactMetaData.xml')
              sh('cd qgroundcontrol; git status; git add .; git commit -a -m "Update TC Firmware metadata `date`" || true')
              sh('cd qgroundcontrol; git push origin master || true')
              sh('rm -rf qgroundcontrol')
            }
          }
          when {
            anyOf {
              branch 'main'
              branch 'master' // should be removed, but in case there is something going on...
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

        stage('TC ROS msgs') {
          agent {
            docker { image 'tcio/tc-dev-base-focal:2021-08-18' }
          }
          steps {
            sh('export')
            sh('make distclean; git clean -ff -x -d .')
            withCredentials([usernamePassword(credentialsId: 'tcbuildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh("git clone https://${GIT_USER}:${GIT_PASS}@github.com/TC/tc_msgs.git")
              // 'main' branch
              sh('rm -f tc_msgs/msg/*.msg')
              sh('cp msg/*.msg tc_msgs/msg/')
              sh('cd tc_msgs; git status; git add .; git commit -a -m "Update message definitions `date`" || true')
              sh('cd tc_msgs; git push origin main || true')
              sh('rm -rf tc_msgs')
            }
          }
          when {
            anyOf {
              branch 'main'
            }
          }
        }

        stage('S3') {
          agent {
            docker { image 'tcio/tc-dev-base-focal:2021-08-18' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            sh('ls')
            withAWS(credentials: 'tc_aws_s3_key', region: 'us-east-1') {
              s3Upload(acl: 'PublicRead', bucket: 'tc-travis', file: 'airframes.xml', path: 'Firmware/master/')
              s3Upload(acl: 'PublicRead', bucket: 'tc-travis', file: 'parameters.xml', path: 'Firmware/master/')
              s3Upload(acl: 'PublicRead', bucket: 'tc-travis', file: 'parameters.json.xz', path: 'Firmware/master/')
            }
          }
          when {
            anyOf {
              branch 'main'
              branch 'master' // should be removed, but in case there is something going on...
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

      } // parallel
    } // stage: Generate Metadata

  } // stages

  environment {
    CCACHE_DIR = '/tmp/ccache'
    CI = true
    GIT_AUTHOR_EMAIL = "bot@tc.io"
    GIT_AUTHOR_NAME = "TCBuildBot"
    GIT_COMMITTER_EMAIL = "bot@tc.io"
    GIT_COMMITTER_NAME = "TCBuildBot"
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '20', artifactDaysToKeepStr: '30'))
    timeout(time: 90, unit: 'MINUTES')
  }
}
